#include <gazebo_test_tools/FakeObjectRecognizer.h>
#include <boost/thread.hpp>

#include <iostream>

#define DEFAULT_OBJECTS_TOPIC "world/objects"
#define DEFAULT_SERVICE_REQUEST_OBJECT_TOPIC "world/request_object"
#define DEFAULT_SERVICE_RECOGNISE_OBJECT_TOPIC "/recognize_object"
#define DEFAULT_PUBLISH_RECOGNISED_OBJECT_RATE 1

using gazebo_test_tools::FakeObjectRecognizer;

FakeObjectRecognizer::FakeObjectRecognizer() {

    ros::NodeHandle _node("/gazebo_test_tools");
    _node.param<std::string>("objects_topic", OBJECTS_TOPIC, DEFAULT_OBJECTS_TOPIC);
    ROS_INFO("Got objects topic name: <%s>", OBJECTS_TOPIC.c_str());

    _node.param<std::string>("request_object_service", SERVICE_REQUEST_OBJECT_TOPIC, DEFAULT_SERVICE_REQUEST_OBJECT_TOPIC);
    ROS_INFO("Got object service topic name: <%s>", SERVICE_REQUEST_OBJECT_TOPIC.c_str());
    
    SERVICE_RECOGNISE_OBJECT_TOPIC= DEFAULT_SERVICE_RECOGNISE_OBJECT_TOPIC;
    _node.param<std::string>("recognize_object_service", SERVICE_RECOGNISE_OBJECT_TOPIC,SERVICE_RECOGNISE_OBJECT_TOPIC);

    std::stringstream def_coll_rate;
    def_coll_rate<<DEFAULT_PUBLISH_RECOGNISED_OBJECT_RATE;    
    std::string _PUBLISH_RECOGNISED_OBJECT_RATE=def_coll_rate.str();
    _node.param<std::string>("publish_recognition_rate", _PUBLISH_RECOGNISED_OBJECT_RATE, _PUBLISH_RECOGNISED_OBJECT_RATE);
    PUBLISH_RECOGNISED_OBJECT_RATE=atof(_PUBLISH_RECOGNISED_OBJECT_RATE.c_str());

    ros::NodeHandle nodePub("");
    if (SERVICE_REQUEST_OBJECT_TOPIC!="") object_info_client = nodePub.serviceClient<object_msgs::ObjectInfo>(SERVICE_REQUEST_OBJECT_TOPIC);

    recognize_object_srv = nodePub.advertiseService(SERVICE_RECOGNISE_OBJECT_TOPIC, &FakeObjectRecognizer::recognizeObject,this);

    object_pub = nodePub.advertise<object_msgs::Object>(OBJECTS_TOPIC, 100); 

    ros::Rate rate(PUBLISH_RECOGNISED_OBJECT_RATE);
    publishTimer=nodePub.createTimer(rate,&FakeObjectRecognizer::publishRecognitionEvent, this);
}

FakeObjectRecognizer::~FakeObjectRecognizer() {
}

void FakeObjectRecognizer::publishRecognitionEvent(const ros::TimerEvent& e) {
    if (object_pub.getNumSubscribers()==0) return;
    
    boost::unique_lock<boost::mutex> lock(addedObjectsMtx);
    std::set<std::string>::iterator it;
    for (it=addedObjects.begin(); it!=addedObjects.end(); ++it )
    {
        object_msgs::Object object;
        // get the object information. Only the pose is required because
        // the shape has been published in recognizeObject() already and
        // it only needs to be published once when the object is first added.
        if (!getGazeboObject(*it,object,false))
        {
            ROS_ERROR_STREAM("Could not find Gazebo object "<<*it);
            continue;
        }
        object_pub.publish(object);
    }
}
    
bool FakeObjectRecognizer::recognizeObject(gazebo_test_tools::RecognizeGazeboObject::Request &req,
        gazebo_test_tools::RecognizeGazeboObject::Response &res)
{
    ROS_INFO_STREAM("Recognizing gazebo object "<<req.name); 
    
    res.success=true;
    boost::unique_lock<boost::mutex> lock(addedObjectsMtx);
    std::set<std::string>::iterator it = addedObjects.find(req.name);
    if (it!=addedObjects.end())
    {
        if (req.republish)
        { 
            ROS_WARN_STREAM("The object "<<req.name<<" was already set to being "
                 << "continously published. No need to call FakeObjectRecognizerService again.");
            return true; 
        }
        else  // switch off repbulishing for this object
        {
            ROS_INFO_STREAM("Removing object "<<req.name<<" from being re-published");
            addedObjects.erase(it);
            // now, publish the object information one last time, to
            // make sure this message is not misunderstood: it could have been
            // a caller which wants the message to be published once and wasn't aware
            // that the object was actually being re-published.
        }
    }
    if (req.republish) addedObjects.insert(req.name);

    object_msgs::Object object;
    if (!getGazeboObject(req.name,object,true))
    {
        ROS_ERROR_STREAM("Could not find Gazebo object "<<req.name);
        res.success=false;
        return true;
    }

    ROS_INFO_STREAM("Publishing object "<<object);       
    object_pub.publish(object);
    return true;
}


bool FakeObjectRecognizer::getGazeboObject(const std::string& name, object_msgs::Object& object, bool include_geometry){
    object_msgs::ObjectInfo srv;
    srv.request.name=name;
    srv.request.get_geometry=include_geometry;
    if (!object_info_client.call(srv)){
        ROS_ERROR("Could not get object %s because service request failed.",name.c_str());
        return false;
    }
    if (!srv.response.success) {
        ROS_ERROR("Could not get object %s because it does not exist in Gazebo.",name.c_str());
        return false;
    }
    object=srv.response.object;
    return true;
}


