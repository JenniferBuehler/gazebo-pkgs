#ifndef GAZEBO_GAZEBOGRASPGRIPPER_H
#define GAZEBO_GAZEBOGRASPGRIPPER_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <stdio.h>

namespace gazebo {

/**

 * \author Jennifer Buehler
 */ 
class GazeboGraspGripper {
public:
    GazeboGraspGripper();
    GazeboGraspGripper(const GazeboGraspGripper& o);
    virtual ~GazeboGraspGripper();
   
    /**
     *
     * \param disableCollisionsOnAttach when an object is attached, collisions with it will be disabled. This is useful
     *      if the robot then still keeps wobbling.
     */ 
    bool Init(physics::ModelPtr& _model,
        const std::string& _gripperName,
        const std::string& palmLinkName,
        const std::vector<std::string>& fingerLinkNames,
        bool _disableCollisionsOnAttach,
        std::map<std::string, physics::CollisionPtr>& _collisions);

    const std::string& getGripperName() const
    {
        return gripperName;
    }

    /**
     * Has the link name (URDF)
     */
    bool hasLink(const std::string& linkName) const
    {
        for (std::vector<std::string>::const_iterator it=linkNames.begin(); it!=linkNames.end(); ++it)
        {
            if (*it==linkName) return true;
        }
        return false;
    }

    /**
     * Has the collision link name (Gazebo collision element name)
     */
    bool hasCollisionLink(const std::string& linkName) const
    {
        return collisionElems.find(linkName) != collisionElems.end();
    }


    bool isObjectAttached() const
    {
        return attached;
    }

    const std::string& attachedObject() const
    {
        return attachedObjName; 
    }

    /**
     * \param gripContacts contact forces on the object sorted by the link name colliding.
     */
    bool HandleAttach(const std::string& objName);
    void HandleDetach(const std::string& objName);

private: 

    physics::ModelPtr model;

    // name of the gripper
    std::string gripperName;
    
    // names of the gripper links
    std::vector<std::string> linkNames;
    // names and Collision objects of the collision links in Gazebo (scoped names)
    // Not necessarily equal names and size to linkNames.
    std::map<std::string, physics::CollisionPtr> collisionElems;

    physics::JointPtr fixedJoint;

    physics::LinkPtr palmLink;

    // when an object is attached, collisions with it may be disabled, in case the
    // robot still keeps wobbling.
    bool disableCollisionsOnAttach;

#if 0  
    // tolerance (in degrees) between force vectors to
    // beconsidered "opposing"
    float forcesAngleTolerance;
    

    // list of current collisions per gripper link 
    std::map<std::string, physics::CollisionPtr> collisions;

    // when an object was first attached, it had these colliding points.
    // Key is object name.
    std::map<std::string, std::map<std::string, CollidingPoint> > attachGripContacts; 
   
    // number of recorded "grips" in the past (in gripCount) which, when it is exceeded, counts
    // as the object grasped, and when it is lower, as released.
    int gripCountThreshold;

    // once an object is gripped, the relative position of the collision link surface to the
    // object is remembered. As soon as this distance changes more than release_tolerance,
    // the object is released.
    float releaseTolerance;
#endif

    // flag holding whether an object is attached. Object name in \e attachedObjName
    bool attached;
    // name of the object currently attached.
    std::string attachedObjName;    
};

}

#endif  // GAZEBO_GAZEBOGRASPGRIPPER_H
