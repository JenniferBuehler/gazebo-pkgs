#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>

#include <gazebo_test_tools/gazebo_cube_spawner.h>

#include <sstream>

using gazebo_test_tools::GazeboCubeSpawner;
using ros::NodeHandle;


#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"

GazeboCubeSpawner::GazeboCubeSpawner(NodeHandle &n) : nh(n){
    spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
}


void GazeboCubeSpawner::spawnCube(const std::string& name, const std::string& frame_id, float x, float y, float z, float qx, float qy, float qz, float qw, float width, float height, float depth, float _mass) {

    geometry_msgs::Pose pose;
    pose.position.x=x;
    pose.position.y=y;
    pose.position.z=z;
    pose.orientation.x=qx;
    pose.orientation.y=qy;
    pose.orientation.z=qz;
    pose.orientation.w=qw;

    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name=name;

    float w=width;
    float h=height;
    float d=depth;

    float mass=_mass;
    float mass12=mass/12.0;

    double mu1=500; //500 for PR2 finger tip. In first experiment had it on 1000000
    double mu2=mu1;
    double kp=10000000; //10000000 for PR2 finger tip
    double kd=1; //100 for rubber? 1 fir OR2 finger tip

    bool do_surface=false;
    bool do_inertia=true;

    std::stringstream s;\
    s<<"<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"<<name<<"'>\
        <static>false</static>\
        <link name='link'>";
    if (do_inertia) 
        s<<"<inertial>\
        <mass>"<<mass<<"</mass>\
        <inertia>\
          <ixx>"<<mass12*(h*h+d*d)<<"</ixx>\
          <ixy>0.0</ixy>\
          <ixz>0.0</ixz>\
          <iyy>"<<mass12*(w*w+d*d)<<"</iyy>\
          <iyz>0.0</iyz>\
          <izz>"<<mass12*(w*w+h*h)<<"</izz>\
        </inertia>\
          </inertial>";
         
    s<<"<collision name='collision'>\
        <geometry>\
          <box>\
            <size>"<<w<<" "<<h<<" "<<d<<"</size>\
          </box>\
        </geometry>";
    if (do_surface)
        s<<"<surface>\
            <friction>\
              <ode>\
            <mu>"<<mu1<<"</mu>\
            <mu2>"<<mu2<<"</mu2>\
            <fdir1>0.000000 0.000000 0.000000</fdir1>\
            <slip1>0.000000</slip1>\
            <slip2>0.000000</slip2>\
              </ode>\
            </friction>\
            <bounce>\
              <restitution_coefficient>0.000000</restitution_coefficient>\
              <threshold>100000.000000</threshold>\
            </bounce>\
            <contact>\
              <ode>\
            <soft_cfm>0.000000</soft_cfm>\
            <soft_erp>0.200000</soft_erp>\
            <kp>"<<kp<<"</kp>\
            <kd>"<<kd<<"</kd>\
            <max_vel>100.000000</max_vel>\
            <min_depth>0.001000</min_depth>\
              </ode>\
            </contact>\
        </surface>";
      s<<"</collision>\
          <visual name='visual'>\
        <geometry>\
          <box>\
            <size>"<<w<<" "<<h<<" "<<d<<"</size>\
          </box>\
        </geometry>\
        <material>\
            <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri> \
                <name>Gazebo/Blue</name>\
            </script>\
        </material>\
          </visual>\
        </link>\
      </model>\
    </sdf>";
    
    spawn.request.model_xml=s.str();
    spawn.request.robot_namespace="cube_spawner";
    spawn.request.initial_pose=pose;
    spawn.request.reference_frame=frame_id;

    //ROS_INFO("Resulting model: \n %s",s.str().c_str());

    //ROS_INFO("Waiting for service");
    spawn_object.waitForExistence();
    //ROS_INFO("Calling service");

    //std::cout<<spawn.request<<std::endl;

    if (!spawn_object.call(spawn)) {
        ROS_ERROR("Failed to call service %s",SPAWN_OBJECT_TOPIC);
    }
    ROS_INFO("Result: %s, code %u",spawn.response.status_message.c_str(), spawn.response.success);
}
