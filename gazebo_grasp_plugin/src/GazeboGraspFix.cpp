#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
 
#include <gazebo_grasp_plugin/GazeboGraspFix.h>

using gazebo::GazeboGraspFix;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)

/**
 * Helper class to encapsulate a collision information
 */
class GazeboGraspFix::CollidingPoint{
public:
    CollidingPoint(): sum(0) {}
    CollidingPoint(const CollidingPoint& o):
        collLink(o.collLink),
        collObj(o.collObj),
        force(o.force),
        pos(o.pos),
        objPos(o.objPos),
        sum(o.sum){}

    //the collision        
    physics::CollisionPtr collLink, collObj;

    //average force vector of the colliding point
    gazebo::math::Vector3 force;
    //position (relative to reference frame of gripper collision surface) where the contact happens on collision surface
    gazebo::math::Vector3 pos;
    //position (relative to reference frame of gripper collision surface) where the object center is located during collision. 
    gazebo::math::Vector3 objPos;
     
    //sum of force and pose (they are actually summed up from several contact points), divide force and pose by this to obtain average
    int sum;
};




GazeboGraspFix::GazeboGraspFix(){
    InitValues();
}

GazeboGraspFix::GazeboGraspFix(physics::ModelPtr _model){
    this->model = _model;
    InitValues();
}

GazeboGraspFix::~GazeboGraspFix() {
    /**
    //XXX this is for gazebo 4.0
      if (this->world && this->model && this->world->GetRunning()) {
        physics::ContactManager *mgr = this->world->GetPhysicsEngine()->GetContactManager();
        mgr->RemoveFilter(this->model->GetScopedName());
    }
    */

    this->model.reset();
    this->physics.reset();
    this->update_connection.reset();
    if (this->node) this->node->Fini();
    this->node.reset();
}

void GazeboGraspFix::Init(){
    this->prevUpdateTime = common::Time::GetWallTime();
}

void GazeboGraspFix::InitValues(){

    float timeDiff=0.25;
    this->release_tolerance=0.005;
    this->attached = false;
    this->updateRate = common::Time(0, common::Time::SecToNano(timeDiff));
    this->prevUpdateTime = common::Time::GetWallTime();


    float graspedSecs=2;
    this->max_grip_count=floor(graspedSecs/timeDiff);    
    this->grip_count_threshold=floor(max_grip_count/2);

    this->attached = false;

    this->node = transport::NodePtr(new transport::Node());
}


void GazeboGraspFix::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->physics = this->model->GetWorld()->GetPhysicsEngine();
    this->fixedJoint = this->physics->CreateJoint("revolute");

    this->node->Init(this->model->GetWorld()->GetName());
        
    physics::ContactManager * contactManager = this->physics->GetContactManager();
    contactManager->PublishContacts(); //XXX not sure I need this?

    sdf::ElementPtr handLinkElem = _sdf->GetElement("palm_link");
    sdf::ElementPtr fingerLinkElem = _sdf->GetElement("gripper_link");

    if (!handLinkElem.get() || !fingerLinkElem.get()) {
        gzerr<<"Cannot use GazeboGraspFix model plugin because no <palm_link> and/or <gripper_link> elements specified in URDF/SDF."<<std::endl;
        return;
    }

      this->palmLink = this->model->GetLink(handLinkElem->Get<std::string>());
    
    std::vector<std::string> collisionScopedNames;

    while (fingerLinkElem) {
        physics::LinkPtr link = this->model->GetLink(fingerLinkElem->Get<std::string>());
        //std::cout<<"Got link "<<fingerLinkElem->Get<std::string>()<<std::endl;
        if (!link.get()){
            gzerr<<"Link "<<fingerLinkElem->Get<std::string>()<<" specified in Model URDF can't be found in gazebo for GazeboGraspFix model plugin. Skipping."<<std::endl;
            fingerLinkElem = fingerLinkElem->GetNextElement("gripper_link");
            continue;
        }
        for (unsigned int j = 0; j < link->GetChildCount(); ++j) {
            physics::CollisionPtr collision = link->GetCollision(j);
            std::string scopedName = collision->GetScopedName();

            std::map<std::string, physics::CollisionPtr>::iterator collIter = this->collisions.find(scopedName);
            if (collIter != this->collisions.end()) { //this collision was already added before
                continue;
            }

            /*collision->SetContactsEnabled(true);
            this->connections.push_back(collision->ConnectContact(boost::bind(&GazeboGraspFix::OnContact, this, _1, _2)));*/
            std::cout<<"GazeboGraspFix: Adding collision scoped name "<<scopedName<<std::endl;
            
            this->collisions[scopedName] = collision;
            collisionScopedNames.push_back(scopedName);
        }
        fingerLinkElem = fingerLinkElem->GetNextElement("gripper_link");
    }

    std::string topic = contactManager->CreateFilter(this->model->GetScopedName(), collisionScopedNames);
    if (!this->contactSub.get()) {
        std::cout<<"Subscribing contact manager to topic "<<topic<<std::endl;
        bool latching=false;
        this->contactSub = this->node->Subscribe(topic,&GazeboGraspFix::OnContact, this, latching);
    }

    update_connection=event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboGraspFix::OnUpdate, this));
}

void GazeboGraspFix::OnUpdate() {

    //std::cout<<"UPDATE!"<<std::endl;

    if (common::Time::GetWallTime() - this->prevUpdateTime < this->updateRate)
        return;

    //first, copy all contact data into local struct. Don't do the complex grip check (checkGrip)
    //within the mutex, because that slows down OnContact().
    this->mutexContacts.lock();
    std::map<std::string, std::map<std::string, CollidingPoint> > contPoints(contacts);
    this->contacts.clear();
    this->mutexContacts.unlock();

    //Gather all summed forces for all the objects (so we have all forces on one object). 
    std::map<std::string, std::vector<gazebo::math::Vector3> > appliedForces; //sorted by object name, all forces effecting on the object
    std::map<std::string, std::map<std::string, CollidingPoint> >::iterator objIt;
    for (objIt=contPoints.begin(); objIt!=contPoints.end(); ++objIt){
        std::string objName=objIt->first;
        //std::cout<<"Examining object collisions with "<<objName<<std::endl;
        std::map<std::string, CollidingPoint>::iterator lIt;
        
        std::vector<gazebo::math::Vector3> forces;
        for (lIt=objIt->second.begin(); lIt!=objIt->second.end(); ++lIt){
            std::string linkName=lIt->first;
            CollidingPoint& collP=lIt->second;
            gazebo::math::Vector3 avgForce=collP.force/collP.sum;
            //std::cout<<"Found collision with "<<linkName<<": "<<avgForce.x<<", "<<avgForce.y<<", "<<avgForce.z<<" (avg over "<<collP.sum<<")"<<std::endl;
            forces.push_back(avgForce);
        }
        appliedForces[objName]=forces;
    }


    std::set<std::string> grippedObjects;    

    std::map<std::string, std::vector<gazebo::math::Vector3> >::iterator it;
    for (it=appliedForces.begin(); it!=appliedForces.end(); ++it) {
        
        std::string objName=it->first;
        float minAngleDiff=120 * M_PI/180;
    
        //std::cout<<"Number applied forces on "<<objName<<": "<<it->second.size()<<std::endl;
        
        if (checkGrip(it->second,minAngleDiff,0.3)) {
            int& counts=gripCounts[objName];
            
            if (counts < max_grip_count) ++counts;                
            
            //std::cout<<"GRIPPING "<<objName<<", grip count "<<counts<<" (threshold "<<grip_count_threshold<<") !!!!!!! "<<std::endl;
        

            if (attached && (attachedObjName != objName)) {
                //if (counts > grip_count_threshold) 
                    gzwarn<<"GazeboGraspFix has found that object "<<
                        attachedObjName<<" is already attached, can't grasp "<<objName<<" as well !"<<std::endl;
                continue;
            }
            
            if (!attached && (counts > grip_count_threshold))  {
                std::cout<<"GazeboGraspFix: Attaching "<<objName<<" !!!!!!!"<<std::endl;
                //for this grip, backup the array of contact poses which played part in the grip, sorted by colliding link
                attachGripContacts[objName]=contPoints[objName];
                if (this->HandleAttach(objName)){
                    attached=true;
                    attachedObjName=objName;
                }
            }
            grippedObjects.insert(objName);
        }
    }
    


    //now, for all objects that are not gripped, decrease grip counter, and possibly release object.
    std::map<std::string, int>::iterator gripCntIt;
    for (gripCntIt=gripCounts.begin(); gripCntIt!=gripCounts.end(); ++gripCntIt){

        std::string objName=gripCntIt->first;

        if (grippedObjects.find(objName)!=grippedObjects.end()) continue;
        
        //std::cout<<"NOT-GRIPPING "<<objName<<", grip count "<<gripCntIt->second<<" (threshold "<<grip_count_threshold<<") !!!!!!! "<<std::endl;

        if (gripCntIt->second > 0) --(gripCntIt->second);

        if (!attached || (gripCntIt->second > grip_count_threshold)) continue;

        //object should potentially be detached now. However, this happens too easily when just considering the count, as the fingers
        //in gazebo start wobbling as the arm moves around, and albeit they are still close to the object, the grip is not 
        //detected any more. So to be sure, we will check that the collision point has not moved too far from where it originally was.

        std::map<std::string, std::map<std::string, CollidingPoint> >::iterator collIt=attachGripContacts.find(objName);
        if (collIt==attachGripContacts.end()) {
            gzerr<<"Consistency: Could not find attachGripContacts for "<<objName<<std::endl;
            continue;
        }

        std::map<std::string, CollidingPoint>& colls=collIt->second;
        int cntRelease=0;
        
        std::map<std::string, CollidingPoint>::iterator pointIt;
        for (pointIt=colls.begin(); pointIt!=colls.end(); ++pointIt) {

            CollidingPoint& cpInfo=pointIt->second;
            gazebo::math::Vector3 relContactPos=cpInfo.pos/cpInfo.sum;
            gazebo::math::Vector3 relObjPos=cpInfo.objPos/cpInfo.sum;
            
            gazebo::math::Pose currObjWorldPose=cpInfo.collObj->GetLink()->GetWorldPose();

            //now, get the contact location on the collision surface in world coordinates now.
            gazebo::math::Pose currCollWorldPose=cpInfo.collLink->GetLink()->GetWorldPose();
            gazebo::math::Matrix4 worldToCollision=currCollWorldPose.rot.GetAsMatrix4();
            worldToCollision.SetTranslate(currCollWorldPose.pos);

            //transform from collision link to contact point
            gazebo::math::Matrix4 collToContact=gazebo::math::Matrix4::IDENTITY;
            collToContact.SetTranslate(relContactPos);
                    
            //now, current world position of the contact point is:
            gazebo::math::Matrix4 _currContactWorldPose=worldToCollision*collToContact;
            gazebo::math::Vector3 currContactWorldPose=_currContactWorldPose.GetTranslation();

            gazebo::math::Vector3 oldObjDist=relContactPos-relObjPos;
            gazebo::math::Vector3 newObjDist=currContactWorldPose-currObjWorldPose.pos;
            
            //std::cout<<"Obj Trans "<<cpInfo.collLink->GetName()<<": "<<relObjPos.x<<", "<<relObjPos.y<<", "<<relObjPos.z<<std::endl;
            //std::cout<<"Cont Trans "<<cpInfo.collLink->GetName()<<": "<<relContactPos.x<<", "<<relContactPos.y<<", "<<relContactPos.z<<std::endl;

            float diff=fabs(oldObjDist.GetLength() - newObjDist.GetLength());
            //std::cout<<"Diff for link "<<cpInfo.collLink->GetName()<<": "<<diff<<std::endl;

            if (diff > release_tolerance) {
                ++cntRelease;
            }
        }

        if (cntRelease > 0){ //sufficient links did not meet the criteria to be close enough to the object.
            std::cout<<"GazeboGraspFix: Detaching "<<objName<<" !!!!!!!"<<std::endl;
            this->HandleDetach(objName);
            attached=false;
            attachedObjName="";
            gripCntIt->second=0;
            attachGripContacts.erase(collIt);
        }
    }



    this->prevUpdateTime = common::Time::GetWallTime();

}

    
double angularDistance(const gazebo::math::Vector3& _v1, const gazebo::math::Vector3& _v2) {
    gazebo::math::Vector3 v1=_v1;        
    gazebo::math::Vector3 v2=_v2;
    v1.Normalize();
    v2.Normalize();    
    return acos(v1.Dot(v2));    
}

bool GazeboGraspFix::checkGrip(const std::vector<gazebo::math::Vector3>& forces, float minAngleDiff, float lengthRatio){
    if (((lengthRatio > 1) || (lengthRatio < 0)) && (lengthRatio > 1e-04 && (fabs(lengthRatio-1) > 1e-04)))  {
        gzerr<<"checkGrip: always specify a lengthRatio of [0..1]"<<std::endl;
        return false;
    }
    if (minAngleDiff < M_PI_2){
        gzerr<<"checkGrip: min angle must be at least 90 degrees (PI/2)"<<std::endl;
        return false;
    }
    std::vector<gazebo::math::Vector3>::const_iterator it1, it2;
    for (it1=forces.begin(); it1!=forces.end(); ++it1){
        gazebo::math::Vector3 v1=*it1;
        for (it2=it1+1; it2!=forces.end(); ++it2){
            gazebo::math::Vector3 v2=*it2;
            float l1=v1.GetLength();
            float l2=v2.GetLength();
            if ((l1<1e-04) || (l2<1e-04)) continue;
            gazebo::math::Vector3 _v1=v1;
            gazebo::math::Vector3 _v2=v2;
            _v1/=l1;
            _v2/=l2;
            float angle=acos(_v1.Dot(_v2));
            if (angle > minAngleDiff) {
                float ratio;
                if (l1>l2) ratio=l2/l1;
                else ratio=l1/l2;
                if (ratio >= lengthRatio) return true;
            }            
        }
    }
}


void GazeboGraspFix::OnContact(const ConstContactsPtr &_msg) {

    
    //std::cout<<"CONTACT! "<<std::endl;//<<_contact<<std::endl;
    for (int i = 0; i < _msg->contact_size(); ++i) {
        physics::CollisionPtr collision1 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->GetEntity(_msg->contact(i).collision1()));
        physics::CollisionPtr collision2 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->GetEntity(_msg->contact(i).collision2()));

        if ((collision1 && !collision1->IsStatic()) && (collision2 && !collision2->IsStatic())) {
            
            std::string name1 = collision1->GetScopedName();
            std::string name2 = collision2->GetScopedName();
            

            //std::cout<<"OBJ CONTACT! "<<name1<<" / "<<name2<<std::endl;

            int count = _msg->contact(i).position_size();

            // Check to see if the contact arrays all have the same size.
            if ((count != _msg->contact(i).normal_size()) ||
                (count != _msg->contact(i).wrench_size()) ||
                (count != _msg->contact(i).depth_size())) {
                gzerr << "GazeboGraspFix: Contact message has invalid array sizes\n";
                continue;
            }

            std::string collidingObjName, collidingLink;
            physics::CollisionPtr linkCollision;
            physics::CollisionPtr objCollision;
            physics::Contact contact;
            contact = _msg->contact(i);

            if (contact.count<1){
                gzerr<<"GazeboGraspFix: Not enough forces given for contact of ."<<name1<<" / "<<name2<<std::endl;
                continue;
            }

            std::vector<gazebo::math::Vector3> force;
            
            //find out which part of the colliding objects is not the gripper
            if (this->collisions.find(name1) == this->collisions.end()) {
                collidingObjName=name1;
                collidingLink=name2;
                linkCollision=collision2;
                objCollision=collision1;
                for (int k=0; k<contact.count; ++k) force.push_back(contact.wrench[k].body1Force);
            }else if (this->collisions.find(name2) == this->collisions.end()) {
                collidingObjName=name2;
                collidingLink=name1;
                linkCollision=collision1;
                objCollision=collision2;
                for (int k=0; k<contact.count; ++k) force.push_back(contact.wrench[k].body2Force);
            }

            gazebo::math::Vector3 sumForce;
            //compute sum of the body wrenches
            for (int k=0; k<force.size(); ++k){
                sumForce+=force[k];
            }    
            sumForce/=force.size();

            gazebo::math::Vector3 avgPos;
            for (int k=0; k<contact.count; ++k) avgPos+=contact.positions[k];
            avgPos/=contact.count;

            //now, get average pose relative to the colliding link
            gazebo::math::Pose collWorldPose=linkCollision->GetLink()->GetWorldPose();

            //To find out the collision point relative to the Link's local coordinate system, first get the Poses as 4x4 matrices
            gazebo::math::Matrix4 worldToCollision=collWorldPose.rot.GetAsMatrix4();
            worldToCollision.SetTranslate(collWorldPose.pos);
            
            gazebo::math::Matrix4 worldToContact=gazebo::math::Matrix4::IDENTITY;
            //we can assume that the contact has identity rotation because we don't care about its orientation.
            //We could always set another rotation here too.
            worldToContact.SetTranslate(avgPos);


            //now, worldToCollision * contactInLocal = worldToContact
            //hence, contactInLocal = worldToCollision.Inv * worldToContact
            gazebo::math::Matrix4 worldToCollisionInv = worldToCollision.Inverse();
            gazebo::math::Matrix4 contactInLocal = worldToCollisionInv * worldToContact;
            gazebo::math::Vector3 contactPosInLocal = contactInLocal.GetTranslation();
            
            //std::cout<<"---------"<<std::endl;    
            //std::cout<<"CNT in loc: "<<contactPosInLocal.x<<","<<contactPosInLocal.y<<","<<contactPosInLocal.z<<std::endl;

            /*gazebo::math::Vector3 sDiff=avgPos-collWorldPose.pos;
            std::cout<<"SIMPLE trans: "<<sDiff.x<<","<<sDiff.y<<","<<sDiff.z<<std::endl;
            std::cout<<"coll world pose: "<<collWorldPose.pos.x<<", "<<collWorldPose.pos.y<<", "<<collWorldPose.pos.z<<std::endl; 
            std::cout<<"contact avg pose: "<<avgPos.x<<", "<<avgPos.y<<", "<<avgPos.z<<std::endl; 

            gazebo::math::Vector3 lX=collWorldPose.rot.GetXAxis();    
            gazebo::math::Vector3 lY=collWorldPose.rot.GetYAxis();    
            gazebo::math::Vector3 lZ=collWorldPose.rot.GetZAxis();    
    
            std::cout<<"World ori: "<<collWorldPose.rot.x<<","<<collWorldPose.rot.y<<","<<collWorldPose.rot.z<<","<<collWorldPose.rot.w<<std::endl;
            std::cout<<"x axis: "<<lX.x<<","<<lX.y<<","<<lX.z<<std::endl;
            std::cout<<"y axis: "<<lY.x<<","<<lY.y<<","<<lY.z<<std::endl;
            std::cout<<"z axis: "<<lZ.x<<","<<lZ.y<<","<<lZ.z<<std::endl;*/

            //now, get the pose of the object and compute it's relative position to the collision surface.
            gazebo::math::Pose objWorldPose=objCollision->GetLink()->GetWorldPose();
            gazebo::math::Matrix4 worldToObj=objWorldPose.rot.GetAsMatrix4();
            worldToObj.SetTranslate(objWorldPose.pos);
    
            gazebo::math::Matrix4 objInLocal = worldToCollisionInv * worldToObj;
            gazebo::math::Vector3 objPosInLocal = objInLocal.GetTranslation();

            {
                boost::mutex::scoped_lock lock(this->mutexContacts);
                CollidingPoint& p=contacts[collidingObjName][collidingLink];
                p.collLink=linkCollision;
                p.collObj=objCollision;
                p.force+=sumForce;
                p.pos+=contactPosInLocal;
                p.objPos+=objPosInLocal;
                p.sum++;
            }
            //std::cout<<"Average force of contact= "<<sumForce.x<<", "<<sumForce.y<<", "<<sumForce.z<<" out of "<<force.size()<<" vectors."<<std::endl;
                    
        }
    }
}


bool GazeboGraspFix::HandleAttach(const std::string& objName) {
    if (!this->palmLink) {
        gzwarn << "palm link not found, not enforcing grasp hack!\n";
        return false;
    }
    physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(objName));
    if (!obj.get()){
        gzerr<<"Object "<<objName<<" not found in world, can't attach it"<<std::endl;
        return false;
    }


    gazebo::math::Pose diff = obj->GetLink()->GetWorldPose() - this->palmLink->GetWorldPose();
    this->fixedJoint->Load(this->palmLink,obj->GetLink(), diff);
    this->fixedJoint->Init();
    this->fixedJoint->SetHighStop(0, 0);
    this->fixedJoint->SetLowStop(0, 0);

    //we should disable collisions of the grasped object, because when the fingers keep colliding with
    //it, the fingers keep wobbling, which can create difficulties when moving the arm. 
    obj->GetLink()->SetCollideMode("none");
    return true;
}

void GazeboGraspFix::HandleDetach(const std::string& objName){
    
    physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(objName));
    if (!obj.get()){
        gzerr<<"Object "<<objName<<" not found in world, can't attach it"<<std::endl;
        return;
    }else{
        obj->GetLink()->SetCollideMode("all");
    }
    
    this->fixedJoint->Detach();

}





