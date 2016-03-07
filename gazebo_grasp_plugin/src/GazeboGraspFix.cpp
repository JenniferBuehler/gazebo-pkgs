#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
 
#include <gazebo_grasp_plugin/GazeboGraspFix.h>

using gazebo::GazeboGraspFix;


#define DEFAULT_FORCES_ANGLE_TOLERANCE 120
#define DEFAULT_UPDATE_RATE 5
#define DEFAULT_MAX_GRIP_COUNT 10    
#define DEFAULT_RELEASE_TOLERANCE 0.005
#define DEFAULT_DISABLE_COLLISIONS_ON_ATTACH false

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)

/**
 * Helper class to encapsulate a collision information.
 * One contact has two bodies, and only
 * the ones where one of the bodies is a gripper link are considered.
 * Each contact consists of a *list* of forces with their own origin/position each
 * (e.g. when the object and gripper are colliding at several places).
 * The averages of each contact's force vectors along with their origins are
 * *accumulated* in the given Vector3 \e pos and \eforce objects.
 * The number of additions is stored in \e sum.
 * This is to get the average force application over time between link and object.
 * 
 * \author Jennifer Buehler
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

    // the collision        
    physics::CollisionPtr collLink, collObj;

    // average force vector of the colliding point
    gazebo::math::Vector3 force;

    // position (relative to reference frame of gripper
    // collision surface) where the contact happens on collision surface
    gazebo::math::Vector3 pos;

    // position (relative to reference frame of *gripper* collision surface)
    // where the object center is located during collision. 
    gazebo::math::Vector3 objPos;
     
    // sum of force and pose (they are actually summed
    // up from several contact points).
    // Divide both \e force and \e pos by this to obtain average
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

    // float timeDiff=0.25;
    //this->releaseTolerance=0.005;
    this->attached = false;
    // this->updateRate = common::Time(0, common::Time::SecToNano(timeDiff));
    this->prevUpdateTime = common::Time::GetWallTime();
    //float graspedSecs=2;
    //this->maxGripCount=floor(graspedSecs/timeDiff);    
    //this->gripCountThreshold=floor(this->maxGripCount/2);

    this->attached = false;
    this->node = transport::NodePtr(new transport::Node());
}


void GazeboGraspFix::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::cout<<"Loading grasp-fix plugin"<<std::endl;

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
        std::cerr<<"ERROR: GazeboGraspFix: Cannot use GazeboGraspFix model plugin because no <palm_link> and/or <gripper_link> elements specified in URDF/SDF."<<std::endl;
        return;
    }

    sdf::ElementPtr forcesAngleToleranceElem = _sdf->GetElement("forces_angle_tolerance");
    if (!forcesAngleToleranceElem.get()){
        std::cout<<"GazeboGraspFix: Using default tolerance of "<<DEFAULT_FORCES_ANGLE_TOLERANCE<<" because no <forces_angle_tolerance> element specified."<<std::endl;
        this->forcesAngleTolerance = DEFAULT_FORCES_ANGLE_TOLERANCE * M_PI/180;
    } else {
        this->forcesAngleTolerance = forcesAngleToleranceElem->Get<float>() * M_PI/180;
    }

    sdf::ElementPtr updateRateElem = _sdf->GetElement("update_rate");
    double _updateSecs;
    if (!updateRateElem.get()){
        std::cout<<"GazeboGraspFix: Using  "<<DEFAULT_UPDATE_RATE<<" because no <updateRate_tag> element specified."<<std::endl;
        _updateSecs = 1.0 / DEFAULT_UPDATE_RATE;
    } else {
        int _rate = updateRateElem->Get<int>();
        double _updateRate = _rate;
        _updateSecs = 1.0/_updateRate;
        std::cout<<"GazeboGraspFix: Using update rate "<<_rate<<std::endl;
    }
    this->updateRate = common::Time(0, common::Time::SecToNano(_updateSecs));

    sdf::ElementPtr maxGripCountElem = _sdf->GetElement("max_grip_count");
    if (!maxGripCountElem.get()){
        std::cout<<"GazeboGraspFix: Using  "<<DEFAULT_MAX_GRIP_COUNT<<" because no <max_grip_count> element specified."<<std::endl;
        this->maxGripCount = DEFAULT_MAX_GRIP_COUNT;
    } else {
        this->maxGripCount = maxGripCountElem->Get<int>();
        std::cout<<"GazeboGraspFix: Using max_grip_count "<<this->maxGripCount<<std::endl;
    }

    sdf::ElementPtr gripCountThresholdElem = _sdf->GetElement("grip_count_threshold");
    if (!gripCountThresholdElem.get()){
        this->gripCountThreshold=floor(this->maxGripCount/2.0);
        std::cout<<"GazeboGraspFix: Using  "<<this->gripCountThreshold<<" because no <grip_count_threshold> element specified."<<std::endl;
    } else {
        this->gripCountThreshold = gripCountThresholdElem->Get<int>();
        std::cout<<"GazeboGraspFix: Using grip_count_threshold "<<this->gripCountThreshold<<std::endl;
    }

    sdf::ElementPtr releaseToleranceElem = _sdf->GetElement("release_tolerance");
    if (!releaseToleranceElem.get()){
        std::cout<<"GazeboGraspFix: Using  "<<DEFAULT_RELEASE_TOLERANCE<<" because no <release_tolerance> element specified."<<std::endl;
        this->releaseTolerance = DEFAULT_RELEASE_TOLERANCE;
    } else {
        this->releaseTolerance = releaseToleranceElem->Get<float>();
        std::cout<<"GazeboGraspFix: Using release_tolerance "<<this->releaseTolerance<<std::endl;
    }

    sdf::ElementPtr disableCollisionsOnAttachElem = _sdf->GetElement("disable_collisions_on_attach");
    if (!disableCollisionsOnAttachElem.get()){
        std::cout<<"GazeboGraspFix: Using  "<<DEFAULT_DISABLE_COLLISIONS_ON_ATTACH<<" because no <disable_collisions_on_attach> element specified."<<std::endl;
        this->disableCollisionsOnAttach = DEFAULT_DISABLE_COLLISIONS_ON_ATTACH;
    } else {
        this->disableCollisionsOnAttach = disableCollisionsOnAttachElem->Get<bool>();
        std::cout<<"GazeboGraspFix: Using disable_collisions_on_attach "<<this->disableCollisionsOnAttach<<std::endl;
    }


    this->palmLink = this->model->GetLink(handLinkElem->Get<std::string>());

    std::vector<std::string> collisionScopedNames;

    while (fingerLinkElem) {
        physics::LinkPtr link = this->model->GetLink(fingerLinkElem->Get<std::string>());
        //std::cout<<"Got link "<<fingerLinkElem->Get<std::string>()<<std::endl;
        if (!link.get()){
            std::cerr<<"ERROR: Link "<<fingerLinkElem->Get<std::string>()<<" specified in Model URDF can't be found in gazebo for GazeboGraspFix model plugin. Skipping."<<std::endl;
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
    if ((common::Time::GetWallTime() - this->prevUpdateTime) < this->updateRate)
        return;

    // first, copy all contact data into local struct. Don't do the complex grip check (checkGrip)
    // within the mutex, because that slows down OnContact().
    this->mutexContacts.lock();
    std::map<std::string, std::map<std::string, CollidingPoint> > contPoints(this->contacts);
    this->contacts.clear(); // clear so it can be filled anew by OnContact().
    this->mutexContacts.unlock();

    // contPoints now contains CollidingPoint objects for each *object* and *link*.

    // Gather all summed forces for all the objects (so we have all forces on one object). 
    std::map<std::string, std::vector<gazebo::math::Vector3> > appliedForces; //sorted by object name, all forces effecting on the object
    std::map<std::string, std::map<std::string, CollidingPoint> >::iterator objIt;
    for (objIt=contPoints.begin(); objIt!=contPoints.end(); ++objIt)
    {
        std::string objName=objIt->first;
        //std::cout<<"Examining object collisions with "<<objName<<std::endl;
        std::map<std::string, CollidingPoint>::iterator lIt;
       
        // for all links colliding with this object... 
        std::vector<gazebo::math::Vector3> forces;
        for (lIt=objIt->second.begin(); lIt!=objIt->second.end(); ++lIt){
            std::string linkName=lIt->first;
            CollidingPoint& collP=lIt->second;
            gazebo::math::Vector3 avgForce=collP.force/collP.sum;
            // std::cout << "Found collision with "<<linkName<<": "<<avgForce.x<<", "<<avgForce.y<<", "<<avgForce.z<<" (avg over "<<collP.sum<<")"<<std::endl;
            forces.push_back(avgForce);
        }
        appliedForces[objName]=forces;
    }

    // XXX TODO: We actually don't currently need this, because only one object can
    // be attached at this stage! This must be a remnant of a previous version of the plugin,
    // unless I'm missing something here. Leaving it in for now.
    std::set<std::string> grippedObjects;    

    std::map<std::string, std::vector<gazebo::math::Vector3> >::iterator it;
    for (it=appliedForces.begin(); it!=appliedForces.end(); ++it) {
        
        std::string objName=it->first;
        float minAngleDiff= this->forcesAngleTolerance; //120 * M_PI/180;
    
        // std::cout<<"Number applied forces on "<<objName<<": "<<it->second.size()<<std::endl;
        
        if (checkGrip(it->second, minAngleDiff, 0.3)) {
            int& counts = this->gripCounts[objName];
            
            if (counts < this->maxGripCount) ++counts;                
            
            //std::cout<<"GRIPPING "<<objName<<", grip count "<<counts<<" (threshold "<<this->gripCountThreshold<<") !!!!!!! "<<std::endl;

            if (this->attached && (this->attachedObjName != objName)) {
                //if (counts > this->gripCountThreshold) 
                    gzwarn<<"GazeboGraspFix has found that object "<<
                        this->attachedObjName<<" is already attached, can't grasp "<<objName<<" as well !"<<std::endl;
                continue;
            }
            
            if (!this->attached && (counts > this->gripCountThreshold))  {
                std::cout<<"GazeboGraspFix: Attaching "<<objName<<" !!!!!!!"<<std::endl;
                //for this grip, backup the array of contact poses which played part in the grip, sorted by colliding link
                this->attachGripContacts[objName]=contPoints[objName];
                if (this->HandleAttach(objName)){
                    this->attached=true;
                    this->attachedObjName=objName;
                }
            }
            grippedObjects.insert(objName);
        }
    }


    // now, for all objects that are not currently gripped,
    // decrease grip counter, and possibly release object.
    std::map<std::string, int>::iterator gripCntIt;
    for (gripCntIt = this->gripCounts.begin(); gripCntIt != this->gripCounts.end(); ++gripCntIt){

        std::string objName=gripCntIt->first;

        if (grippedObjects.find(objName) != grippedObjects.end())
        {   // this object is one we just detected as "gripped", so no need to check for releasing it...
            // std::cout<<"NOT considering "<<objName<<" for detachment."<<std::endl;
            continue;
        }
        
        // the object does not satisfy "gripped" criteria, so potentially has to be released.

        //std::cout<<"NOT-GRIPPING "<<objName<<", grip count "<<gripCntIt->second<<" (threshold "<<this->gripCountThreshold<<") !!!!!!! "<<std::endl;

        if (gripCntIt->second > 0) --(gripCntIt->second);

        if (!this->attached || (gripCntIt->second > this->gripCountThreshold)) continue;
        
        // std::cout<<"Considering "<<objName<<" for detachment."<<std::endl;

        // Object should potentially be detached now.
        // However, this happens too easily when just considering the count, as the fingers
        // in gazebo start wobbling as the arm moves around, and although they are still
        // close to the object, the grip is not detected any more. So to be sure, we will
        // check that the collision point (the place on the link where the contact originally
        // was detected) has not moved too far from where it originally was, relative to the object.

        // get the initial set of CollidingPoints for this object
        // note that it is enough to use the initial contact points, because the object won't
        // have "slipped" after being attached, and the location of the original contact point
        // on the link itself is considered as a fixed point on the link, regardless whether this
        // point is currently still colliding with the object or not. We only want to know whether
        // this fixed point on the link has increased in distance from the corresponding fixed
        // point (where the contact originally happened) on the object.
        std::map<std::string, std::map<std::string, CollidingPoint> >::iterator initCollIt=this->attachGripContacts.find(objName);
        if (initCollIt == this->attachGripContacts.end()) {
            std::cerr<<"ERROR: Consistency: Could not find attachGripContacts for "<<objName<<std::endl;
            continue;
        }

        std::map<std::string, CollidingPoint>& initColls=initCollIt->second;
        int cntRelease=0;
       
        // for all links which have initially been detected to collide: 
        std::map<std::string, CollidingPoint>::iterator pointIt;
        for (pointIt=initColls.begin(); pointIt!=initColls.end(); ++pointIt)
        {
            CollidingPoint& cpInfo=pointIt->second;
            // initial distance from link to contact point (relative to link)
            gazebo::math::Vector3 relContactPos=cpInfo.pos/cpInfo.sum;
            // initial distance from link to object (relative to link)
            gazebo::math::Vector3 relObjPos=cpInfo.objPos/cpInfo.sum;
           
            // get current world pose of object 
            gazebo::math::Pose currObjWorldPose=cpInfo.collObj->GetLink()->GetWorldPose();

            // get world pose of link
            gazebo::math::Pose currLinkWorldPose=cpInfo.collLink->GetLink()->GetWorldPose();

            // Get transform for currLinkWorldPose as matrix
            gazebo::math::Matrix4 worldToLink=currLinkWorldPose.rot.GetAsMatrix4();
            worldToLink.SetTranslate(currLinkWorldPose.pos);

            // Get the transform from collision link to contact point
            gazebo::math::Matrix4 linkToContact=gazebo::math::Matrix4::IDENTITY;
            linkToContact.SetTranslate(relContactPos);
                    
            // the current world position of the contact point right now is:
            gazebo::math::Matrix4 _currContactWorldPose=worldToLink*linkToContact;
            gazebo::math::Vector3 currContactWorldPose=_currContactWorldPose.GetTranslation();

            // the initial contact point location on the link should still correspond
            // to the initial contact point location on the object.

            // initial vector from object center to contact point (relative to link,
            // because relObjPos and relContactPos are from center of link)
            gazebo::math::Vector3 oldObjDist= relContactPos - relObjPos;
            // the same vector as \e oldObjDist, but calculated by the current world pose
            // of object and the current location of the initial contact location on the link.
            gazebo::math::Vector3 newObjDist= currContactWorldPose - currObjWorldPose.pos; // new distance from contact to object
            
            //std::cout<<"Obj Trans "<<cpInfo.collLink->GetName()<<": "<<relObjPos.x<<", "<<relObjPos.y<<", "<<relObjPos.z<<std::endl;
            //std::cout<<"Cont Trans "<<cpInfo.collLink->GetName()<<": "<<relContactPos.x<<", "<<relContactPos.y<<", "<<relContactPos.z<<std::endl;
        
            // the difference between these vectors should not be too large...
            float diff=fabs(oldObjDist.GetLength() - newObjDist.GetLength());
            //std::cout<<"Diff for link "<<cpInfo.collLink->GetName()<<": "<<diff<<std::endl;

            if (diff > releaseTolerance) {
                ++cntRelease;
            }
        }

        if (cntRelease > 0)
        { // sufficient links did not meet the criteria to be close enough to the object.
            std::cout<<"GazeboGraspFix: Detaching "<<objName<<" !!!!!!!"<<std::endl;
            this->HandleDetach(objName);
            this->attached=false;
            this->attachedObjName="";
            gripCntIt->second=0;
            this->attachGripContacts.erase(initCollIt);
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
        std::cerr<<"ERROR: checkGrip: always specify a lengthRatio of [0..1]"<<std::endl;
        return false;
    }
    if (minAngleDiff < M_PI_2){
        std::cerr<<"ERROR: checkGrip: min angle must be at least 90 degrees (PI/2)"<<std::endl;
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
            /*gazebo::math::Vector3 _v1=v1;
            gazebo::math::Vector3 _v2=v2;
            _v1/=l1;
            _v2/=l2;
            float angle=acos(_v1.Dot(_v2));*/
            float angle=angularDistance(v1, v2);
            // std::cout<<"Angular distance between v1.len="<<v1.GetLength()<<" and v2.len="<<v2.GetLength()<<": "<<angle*180/M_PI<<std::endl;
            if (angle > minAngleDiff) {
                float ratio;
                if (l1>l2) ratio=l2/l1;
                else ratio=l1/l2;
                // std::cout<<"Got angle "<<angle<<", ratio "<<ratio<<std::endl;
                if (ratio >= lengthRatio)
                {
                    // std::cout<<"checkGrip() is true"<<std::endl;
                    return true;
                }
            }            
        }
    }
    return false;
}

void GazeboGraspFix::OnContact(const ConstContactsPtr &_msg)
{
    //std::cout<<"CONTACT! "<<std::endl;//<<_contact<<std::endl;
    // for all contacts...
    for (int i = 0; i < _msg->contact_size(); ++i) {
        physics::CollisionPtr collision1 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->GetEntity(_msg->contact(i).collision1()));
        physics::CollisionPtr collision2 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->GetEntity(_msg->contact(i).collision2()));

        if ((collision1 && !collision1->IsStatic()) && (collision2 && !collision2->IsStatic()))
        {
            std::string name1 = collision1->GetScopedName();
            std::string name2 = collision2->GetScopedName();

            //std::cout<<"OBJ CONTACT! "<<name1<<" / "<<name2<<std::endl;
            int count = _msg->contact(i).position_size();

            // Check to see if the contact arrays all have the same size.
            if ((count != _msg->contact(i).normal_size()) ||
                (count != _msg->contact(i).wrench_size()) ||
                (count != _msg->contact(i).depth_size()))
            {
                gzerr << "GazeboGraspFix: Contact message has invalid array sizes\n";
                continue;
            }

            std::string collidingObjName, collidingLink;
            physics::CollisionPtr linkCollision;
            physics::CollisionPtr objCollision;

            physics::Contact contact;
            contact = _msg->contact(i);

            if (contact.count<1)
            {
                std::cerr<<"ERROR: GazeboGraspFix: Not enough forces given for contact of ."<<name1<<" / "<<name2<<std::endl;
                continue;
            }

            // all force vectors which are part of this contact
            std::vector<gazebo::math::Vector3> force;
            
            // find out which part of the colliding entities is the object, *not* the gripper,
            // and copy all the forces applied to it into the vector 'force'.
            if (this->collisions.find(name1) == this->collisions.end())
            {   // collision 1 is the object
                collidingObjName=name1;
                collidingLink=name2;
                linkCollision=collision2;
                objCollision=collision1;
                for (int k=0; k<contact.count; ++k)
                    force.push_back(contact.wrench[k].body1Force);
            }
            else if (this->collisions.find(name2) == this->collisions.end())
            {   // collision 2 is the object
                collidingObjName=name2;
                collidingLink=name1;
                linkCollision=collision1;
                objCollision=collision2;
                for (int k=0; k<contact.count; ++k)
                    force.push_back(contact.wrench[k].body2Force);
            }

            gazebo::math::Vector3 avgForce;
            // compute average/sum of the forces applied on the object
            for (int k=0; k<force.size(); ++k){
                avgForce+=force[k];
            }    
            avgForce/=force.size();

            gazebo::math::Vector3 avgPos;
            // compute center point (average pose) of all the origin positions of the forces appied
            for (int k=0; k<contact.count; ++k) avgPos+=contact.positions[k];
            avgPos/=contact.count;

            // now, get average pose relative to the colliding link
            gazebo::math::Pose linkWorldPose=linkCollision->GetLink()->GetWorldPose();

            // To find out the collision point relative to the Link's local coordinate system, first get the Poses as 4x4 matrices
            gazebo::math::Matrix4 worldToLink=linkWorldPose.rot.GetAsMatrix4();
            worldToLink.SetTranslate(linkWorldPose.pos);
            
            gazebo::math::Matrix4 worldToContact=gazebo::math::Matrix4::IDENTITY;
            //we can assume that the contact has identity rotation because we don't care about its orientation.
            //We could always set another rotation here too.
            worldToContact.SetTranslate(avgPos);

            // now, worldToLink * contactInLocal = worldToContact
            // hence, contactInLocal = worldToLink.Inv * worldToContact
            gazebo::math::Matrix4 worldToLinkInv = worldToLink.Inverse();
            gazebo::math::Matrix4 contactInLocal = worldToLinkInv * worldToContact;
            gazebo::math::Vector3 contactPosInLocal = contactInLocal.GetTranslation();
            
            //std::cout<<"---------"<<std::endl;    
            //std::cout<<"CNT in loc: "<<contactPosInLocal.x<<","<<contactPosInLocal.y<<","<<contactPosInLocal.z<<std::endl;

            /*gazebo::math::Vector3 sDiff=avgPos-linkWorldPose.pos;
            std::cout<<"SIMPLE trans: "<<sDiff.x<<","<<sDiff.y<<","<<sDiff.z<<std::endl;
            std::cout<<"coll world pose: "<<linkWorldPose.pos.x<<", "<<linkWorldPose.pos.y<<", "<<linkWorldPose.pos.z<<std::endl; 
            std::cout<<"contact avg pose: "<<avgPos.x<<", "<<avgPos.y<<", "<<avgPos.z<<std::endl; 

            gazebo::math::Vector3 lX=linkWorldPose.rot.GetXAxis();    
            gazebo::math::Vector3 lY=linkWorldPose.rot.GetYAxis();    
            gazebo::math::Vector3 lZ=linkWorldPose.rot.GetZAxis();    
    
            std::cout<<"World ori: "<<linkWorldPose.rot.x<<","<<linkWorldPose.rot.y<<","<<linkWorldPose.rot.z<<","<<linkWorldPose.rot.w<<std::endl;
            std::cout<<"x axis: "<<lX.x<<","<<lX.y<<","<<lX.z<<std::endl;
            std::cout<<"y axis: "<<lY.x<<","<<lY.y<<","<<lY.z<<std::endl;
            std::cout<<"z axis: "<<lZ.x<<","<<lZ.y<<","<<lZ.z<<std::endl;*/

            // now, get the pose of the object and compute it's relative position to the collision surface.
            gazebo::math::Pose objWorldPose = objCollision->GetLink()->GetWorldPose();
            gazebo::math::Matrix4 worldToObj = objWorldPose.rot.GetAsMatrix4();
            worldToObj.SetTranslate(objWorldPose.pos);
    
            gazebo::math::Matrix4 objInLocal = worldToLinkInv * worldToObj;
            gazebo::math::Vector3 objPosInLocal = objInLocal.GetTranslation();

            {
                boost::mutex::scoped_lock lock(this->mutexContacts);
                CollidingPoint& p = this->contacts[collidingObjName][collidingLink];  // inserts new entry if doesn't exist
                p.collLink=linkCollision;
                p.collObj=objCollision;
                p.force+=avgForce;
                p.pos+=contactPosInLocal;
                p.objPos+=objPosInLocal;
                p.sum++;
            }
            //std::cout<<"Average force of contact= "<<avgForce.x<<", "<<avgForce.y<<", "<<avgForce.z<<" out of "<<force.size()<<" vectors."<<std::endl;
        }
    }
}


// #define USE_MODEL_ATTACH // this only works if the object is a model in itself, which is usually not
                            // the case. Leaving this in here anyway for documentation of what has been
                            // tried, and for and later re-evaluation.
bool GazeboGraspFix::HandleAttach(const std::string& objName)
{
    if (!this->palmLink) {
        gzwarn << "palm link not found, not enforcing grasp hack!\n";
        return false;
    }
#ifdef USE_MODEL_ATTACH
    physics::ModelPtr obj = this->world->GetModel(objName);
    if (!obj.get()){
        std::cerr<<"ERROR: Object ModelPtr "<<objName<<" not found in world, can't attach it"<<std::endl;
        return false;
    }
    gazebo::math::Pose diff = obj->GetLink()->GetWorldPose() - this->palmLink->GetWorldPose();
    this->palmLink->AttachStaticModel(obj,diff);
    this->OnAttach(objName);
#else
    physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(objName));
    if (!obj.get()){
        std::cerr<<"ERROR: Object "<<objName<<" not found in world, can't attach it"<<std::endl;
        return false;
    }
    gazebo::math::Pose diff = obj->GetLink()->GetWorldPose() - this->palmLink->GetWorldPose();
    this->fixedJoint->Load(this->palmLink,obj->GetLink(), diff);
    this->fixedJoint->Init();
    this->fixedJoint->SetHighStop(0, 0);
    this->fixedJoint->SetLowStop(0, 0);
    if (this->disableCollisionsOnAttach) {
        // we can disable collisions of the grasped object, because when the fingers keep colliding with
        // it, the fingers keep wobbling, which can create difficulties when moving the arm. 
        obj->GetLink()->SetCollideMode("none");
    }
    this->OnAttach(objName);
#endif  // USE_MODEL_ATTACH
    return true;
}

void GazeboGraspFix::HandleDetach(const std::string& objName)
{
#ifdef USE_MODEL_ATTACH
     physics::ModelPtr obj = this->world->GetModel(objName);
    if (!obj.get()){
        std::cerr<<"ERROR: Object ModelPtr "<<objName<<" not found in world, can't detach it"<<std::endl;
        return;
    }
    this->palmLink->DetachStaticModel(objName);
    this->OnDetach(objName);
#else
    physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(objName));
    if (!obj.get()){
        std::cerr<<"ERROR: Object "<<objName<<" not found in world, can't attach it"<<std::endl;
        return;
    }
    else if (this->disableCollisionsOnAttach)
    {
        obj->GetLink()->SetCollideMode("all");
    }
    this->fixedJoint->Detach();
    this->OnDetach(objName);
#endif  // USE_MODEL_ATTACH
}
