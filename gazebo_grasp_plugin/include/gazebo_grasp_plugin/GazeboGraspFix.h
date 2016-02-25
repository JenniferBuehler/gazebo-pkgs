#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <stdio.h>

namespace gazebo {

/**
 * Inspired by gazebo::physics::Gripper, this is our own implementation which does not use deprecated functions any more.
 *
 * This is a *model* pluing, so you have to load the model plugin from the robot URDF:
 *
 * ```xml
 *   <gazebo>
 *       <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
 *           <palm_link> hand_link_name  </palm_link>
 *           <gripper_link> finger_index_link_1 </gripper_link>
 *           <gripper_link> finger_index_link_2 </gripper_link>
 *           <gripper_link> ... </gripper_link>
 *       </plugin>
 *   </gazebo>
 * ```
 *
 * \author Jennifer Buehler
 */ 
class GazeboGraspFix : public ModelPlugin {
public:
    GazeboGraspFix();
    GazeboGraspFix(physics::ModelPtr _model);
    virtual ~GazeboGraspFix();
    virtual void Init(); 
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate();
private: 

    void InitValues();
    void OnContact(const ConstContactsPtr& ptr);

    bool HandleAttach(const std::string& objName);
    void HandleDetach(const std::string& objName);

    /**
     * Checks whether any two vectors in the set have an angle greater than minAngleDiff (in rad), and one is at least
     * lengthRatio (0..1)  of the other in it's length.
     */
    bool checkGrip(const std::vector<math::Vector3>& forces, float minAngleDiff, float lengthRatio);
    
    physics::ModelPtr model;

    physics::PhysicsEnginePtr physics;
    physics::WorldPtr world;
    physics::JointPtr fixedJoint;
    
    physics::LinkPtr palmLink;
    event::ConnectionPtr update_connection;
    transport::NodePtr node;
    transport::SubscriberPtr contactSub; //subscriber to contact updates
    
    std::map<std::string, physics::CollisionPtr> collisions;

    /**
     * Helper class to encapsulate a collision information.
     * Forward declaration here.
     */
    class CollidingPoint;

    //Contact forces sorted by object name the gripper collides with, and the link colliding. 
    //This is a vector summed up over time, sum count is kept in contactSumCnt.
    std::map<std::string, std::map<std::string, CollidingPoint> > contacts; 
    boost::mutex mutexContacts; //mutex protects contacts

    //when an object was attached, it had these colliding points.
    std::map<std::string, std::map<std::string, CollidingPoint> > attachGripContacts; 
    
    //records how many update loops (at updateRate) the grip on that object has been recorded 
    //as "holding". Every loop, if a grip is not recorded, this number decreases. 
    //When it reaches grip_count_threshold, it will be attached.
    //The number won't increase above max_grip_count once it has reached that number.
    std::map<std::string, int> gripCounts; 
    //maximum record in gripCounts
    int max_grip_count;    
    //number of recorded "grips" in the past (in gripCount) which, when it is exceeded, counts
    //as the object grasped, and when it is lower, as released.
    int grip_count_threshold;

    //once an object is gripped, the relative position of the collision link surface to the
    //object is remembered. As soon as this distance changes more than release_tolerance,
    //the object is released.
    float release_tolerance;

    bool attached;
    std::string attachedObjName;    

    //nano seconds between two updates
    common::Time updateRate;

    //last time OnUpdate() was called
    common::Time prevUpdateTime;
};

}
