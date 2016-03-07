#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <stdio.h>

namespace gazebo {

/**
 * Inspired by gazebo::physics::Gripper, this plugin fixes an object which is grasped to the
 * robot hand to avoid problems with physics engines and to help the object staying in
 * the robot hand without slipping out.
 *
 * This is a *model* plugin, so you have to load the model plugin from the robot URDF:
 *
 * ```xml
 *   <gazebo>
 *       <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
 *           <!-- Palm link: The link to which the object is attached -->
 *           <palm_link> hand_link_name  </palm_link>
 *
 *           <!-- All gripper links which collide with the object to *actively* grasp it -->
 *           <gripper_link> finger_index_link_1 </gripper_link>
 *           <gripper_link> finger_index_link_2 </gripper_link>
 *           <gripper_link> ... </gripper_link>
 *           <!-- tolerance angle (in degrees) between two force vectors to be considered
 *                "opposing forces". If the angle is smaller than this, they are not opposing.
 *             -->
 *           <forces_angle_tolerance>120</forces_angle_tolerance>
 *
 *           <!-- the rate at which all collision points are checked against the "gripping criterion".
 *              Note that in-between such updates, existing collisions may be collected at
 *              a higher rate (the Gazebo world update rate). This rate is only the rate at
 *              which they are processed, which takes a bit of computation time, and therefore
 *              should be lower than the gazebo world update rate.  -->
 *           <update_rate>5</update_rate>
 *
 *           <!-- Number of times an object has to be detected as "gripped" in order to
 *                attach the object. Adjust this with the update rate. -->
 *           <grip_count_threshold>5</grip_count_threshold>
 *
 *           <!-- Maximum number of times that a "grasped" condition counted for an
 *                object. Should be at least double of grip_count_threshold. If the object
 *                has been counted this amount of times in subsequent update iterations as
 *                "grasped", the number will not increase any further. As soon as the grasp
 *                criterion does not hold any more, the number will start to decrease by one time
 *                each time the object is detected as "not grasped" in an update iteration. This is
 *                like a "buffer" which, when it is full, maintains the state, and when it is empty again,
 *                the object is released -->
 *           <max_grip_count>10</max_grip_count>
 *
 *           <!-- The distance which the gripper links are allowed to move away from the object
 *                *during* a grasp without the object being detached, even if there are currently no
 *                contacts. This can happen if the fingers "wobble" or move ever so slightly, and therefore
 *                the grasp is not detected as active any more. Setting this number too high will also lead
 *                to the object not being detached even if the grippers have opened up to release it. -->
 *           <release_tolerance>0.005</release_tolerance>
 *
 *           <!-- when an object is attached, collisions with it may be disabled, in case the
 *                robot still keeps wobbling. -->  
 *           <disable_collisions_on_attach>false</disable_collisions_on_attach>
 *
 *           <!-- The gazebo topic of contacts. Should normally be left at __default_topic__  -->
 *           <contact_topic>__default_topic__</contact_topic>
 *       </plugin>
 *   </gazebo>
 * ```
 *
 * Methods OnUpdate() and OnContact() contain detailed documentation on how the plugin works.
 * Summarizing it:
 *  XXX TODO
 *
 * Current limitation:
 *  - Only one object can be attached per gripper.
 * \author Jennifer Buehler
 */ 
class GazeboGraspFix : public ModelPlugin {
public:
    GazeboGraspFix();
    GazeboGraspFix(physics::ModelPtr _model);
    virtual ~GazeboGraspFix();
    virtual void Init(); 
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
private: 
    /**
     * Collects for each object all forces which are currently applied on it.
     * Then, for each object, checks whether of all the forces applied,
     * there are opposing forces. This is done by calling checkGrip() with the
     * list of all forces applied.
     * If checkGrip() returns true, the number of "grip counts" (field \e gripCounts)
     * is increased (but grip counts will never exceed \e max_grip_count).
     * If the number of grip counts for this object exceeds \e grip_count_threshold,
     * the object is attached by calling HandleAttach(object-name),
     * setting \e attached and \e attachedObjName, and \e attachGripContacts is updated with the
     * contact points currently existing for this object (current entry in \e contacts).
     *
     * Then, goes through all entries in \e gripCount, and unless it's an object
     * we just detected as "gripped", the counter is decreased.
     * If the counter is is smaller than \e grip_count_threshold, the object should
     * potentially be released, but this criterion happens too easily
     * (the fingers in gazebo may have started wobbling as the arm moves around, and although they are still
     * close to the object, the grip is not detected any more).
     * So to be sure, and additional criterion has to be satisfied before the object is released:
     * check that the collision point (the place on the link where the contact originally
     * was detected) has not moved too far from where it originally was, relative to the object.
     */
    void OnUpdate();

    void InitValues();


    /**
     * Gets called upon detection of contacts.
     * A list of contacts is passed in \_msg. One contact has two bodies, and only
     * the ones where one of the bodies is a gripper link are considered.
     * Each contact consists of a *list* of forces with their own origin/position each
     * (e.g. when the object and gripper are colliding at several places).
     * The averages of each contact's force vectors along with their origins is computed.
     * This "average contact force/origin" for each contact is then added to the \e this->contacts map.
     * If an entry for this object/link pair already exists, the average force (and its origin)
     * is *added* to the existing force/origin, and the average count is increased. This is to get
     * the average force application over time between link and object.  
     */
    void OnContact(const ConstContactsPtr& ptr);

    bool HandleAttach(const std::string& objName);
    void HandleDetach(const std::string& objName);

    /**
     * Checks whether any two vectors in the set have an angle greater
     * than minAngleDiff (in rad), and one is at least
     * lengthRatio (0..1) of the other in it's length.
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
  
    // tolerance (in degrees) between force vectors to
    // beconsidered "opposing"
    float forcesAngleTolerance;
    
    // when an object is attached, collisions with it may be disabled, in case the
    // robot still keeps wobbling.
    bool disableCollisionsOnAttach;
 
    // list of current collisions per gripper link 
    std::map<std::string, physics::CollisionPtr> collisions;

    /**
     * Helper class to encapsulate a collision information.
     * Forward declaration here.
     */
    class CollidingPoint;

    // Contact forces sorted by object name the gripper collides with (first key)
    // and the link colliding (second key). 
    std::map<std::string, std::map<std::string, CollidingPoint> > contacts; 
    boost::mutex mutexContacts; //mutex protects contacts

    // when an object was first attached, it had these colliding points.
    // Key is object name.
    std::map<std::string, std::map<std::string, CollidingPoint> > attachGripContacts; 
    
    // Records how many update loops (at updateRate) the grip on that object has been recorded 
    // as "holding". Every loop, if a grip is not recorded, this number decreases. 
    // When it reaches \e grip_count_threshold, it will be attached.
    // The number won't increase above max_grip_count once it has reached that number.
    std::map<std::string, int> gripCounts; 

    // *maximum* number in \e gripCounts to be recorded.
    int maxGripCount;    

    // number of recorded "grips" in the past (in gripCount) which, when it is exceeded, counts
    // as the object grasped, and when it is lower, as released.
    int gripCountThreshold;

    // once an object is gripped, the relative position of the collision link surface to the
    // object is remembered. As soon as this distance changes more than release_tolerance,
    // the object is released.
    float releaseTolerance;

    // flag holding whether an object is attached. Object name in \e attachedObjName
    bool attached;
    // name of the object currently attached.
    std::string attachedObjName;    

    //nano seconds between two updates
    common::Time updateRate;

    //last time OnUpdate() was called
    common::Time prevUpdateTime;
};

}
