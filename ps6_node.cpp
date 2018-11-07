//Jason Sun, PS6

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>

using namespace std;
//set up the variables for the camera
bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_caml_data;

void cam2CB(const osrf_gear::LogicalCameraImage message_holder){
    ROS_INFO_STREAM("image from caml2: " <<message_holder<<endl);
    g_caml_data = message_holder;
    g_take_new_snapshot = false;
    }

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle n;
    //call the service client of ariac start
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    //call the serice client of the conveyor
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    //call the service client of the drone
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;
    
    //set the startup_srv to false firstly, then the while loop will keep publich the information of not successfully started until the startup_client service is successfully called.
    startup_srv.response.success = false;
while (!startup_srv.response.success) {
    ROS_WARN("not successful, starting up yet....");
    startup_client.call(startup_srv);
    ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from startup service");
    
    //set the moving power of the conveyor to 100 and not start the conveyor yet
    conveyor_srv.request.power = 100;
    conveyor_srv.response.success = false;
    // the loop will keep print the "not successful" information until the conveyor_client service is successfully called
    while (!conveyor_srv.response.success){
        ROS_WARN("not successful starting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
	ros::Duration(8).sleep(); 
    ROS_INFO("got success response from conveyor service");
    
    //camera is set to get the information of how many shipping boxes are passed.
    g_take_new_snapshot = true;
    while(g_caml_data.models.size() < 0){
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("I see a box");

//stop the conveyor for 5 seconds after the comera sees a box. the power would be set to 0 and the while loop would have a ros duration of 5 second in order to keep the conveyor from running for about 5 seconds.
    conveyor_srv.request.power = 0;
   	conveyor_srv.response.success = false;
    while (!conveyor_srv.response.success){
        ROS_WARN("Halting the conveyor for 5 seconds");
 	conveyor_client.call(conveyor_srv);
        ros::Duration(5).sleep();
    }
    ROS_INFO("Conveyor Stoped for 5 seconds");
    //after the five second pause, the conveyor will resume running.
    conveyor_srv.request.power = 100;
    conveyor_srv.response.success = false;
    while (!conveyor_srv.response.success){
        ROS_WARN("Conveyor Resuming");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Conveyor resume");
    
    //After there is a shipping box arrived down to the loading dock, drone_client service would be called and a drone would come and pick up the shipping box. 
    drone_srv.request.shipment_type = "dummy";
    drone_srv.response.success = false;
    while(!drone_srv.response.success){
        ROS_WARN("not successful starting calling drone yet");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("get success response from drone service");
    return 0;
}
