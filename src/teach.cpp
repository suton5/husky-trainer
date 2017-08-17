
#include <vector>
#include <fstream>
#include <math.h>
#include <cmath>
#include <iostream>
#include <unistd.h>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/callback_queue.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "husky_trainer/AnchorPoint.h"
#include "husky_trainer/PointMatching.h"
#include "husky_trainer/NamedPointCloud.h"

#define WORKING_DIRECTORY_PARAM "working_directory"
#define AP_TRIGGER_PARAM "ap_distance"
#define ANGLE_AP_PARAM "ap_angle"
#define DEFAULT_WORKING_DIRECTORY ""  // current working directory

//#define JOYSTICK_TOPIC "/keyop/teleop"
#define KEYBOARD_TOPIC "/ypresser"
#define POINT_CLOUD_TOPIC "/zed/point_cloud/cloud_registered"
#define POSE_ESTIMATE_TOPIC "/odom"
#define VEL_TOPIC "/mobile_base/commands/velocity"
#define CLOUD_RECORDER_TOPIC "/teach_repeat/anchor_points"

#define ROBOT_FRAME "/base_footprint"
#define LIDAR_FRAME "/zed_depth_frame"

#define Y_BUTTON_INDEX 3

#define DEFAULT_AP_TRIGGER 0.1  // The approx distance we want between every anchor point.
#define DEFAULT_AP_ANGLE 0.01
#define LOOP_RATE 100
#define L_SEP ","

typedef PointMatcher<float> PM;

// Node global variables.
std::string workingDirectory;
std::ofstream* pPositionRecord;
std::ofstream* pSpeedRecord;

int nextCloudIndex;
float distanceTravelled;
float lastYawRecorded;

double distanceBetweenAnchorPoints;
double angleBetweenAnchorPoints;

geometry_msgs::Pose poseOfLastAnchor;
geometry_msgs::Pose lastPoseRecorded;

// Path recording information.
ros::Time teachingStartTime;
std::vector<AnchorPoint> anchorPointList;
boost::mutex anchorPointListMutex;
geometry_msgs::Pose lastOdomPosition;
geometry_msgs::Pose prevOdomPosition;
PM::TransformationParameters tLidarToBaseLink;
ros::Publisher* pCloudRecorderTopic;

void saveAnchorPointList(std::vector<AnchorPoint>& list)
{
    std::ofstream anchorPointListFile;
    anchorPointListFile.open("anchorPoints.apd");

    for(int i=0; i < anchorPointList.size(); i++)
    {
        anchorPointListFile << list[i];
    }

    anchorPointListFile.close();
}

void recordCloud(const sensor_msgs::PointCloud2& msg)
{

    PM::DataPoints dataPoints;
    dataPoints = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(msg);

    pointmatching_tools::applyTransform(dataPoints, tLidarToBaseLink);

    husky_trainer::NamedPointCloud namedCloud;
    namedCloud.cloud =
        PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
            dataPoints,
            ROBOT_FRAME,
            ros::Time::now()
        );

    // Create the name of the point cloud.
    std::stringstream ss;
    ss.fill('0');
    ss << std::setw(5) << nextCloudIndex++ << ".vtk";
    namedCloud.name = ss.str();

    pCloudRecorderTopic->publish(namedCloud);
    ROS_INFO("Recorded a new cloud.");

    AnchorPoint newAnchorPoint(namedCloud.name, lastOdomPosition);

    while(!anchorPointListMutex.try_lock()) {}
    anchorPointList.push_back(newAnchorPoint);
    anchorPointListMutex.unlock();
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    double distance_since_ap = geo_util::euclidian_distance_of_poses(lastOdomPosition, poseOfLastAnchor);
    double angle_since_ap = geo_util::angle_between_poses(lastOdomPosition, poseOfLastAnchor);

    //ROS_INFO("Travel: %f", distance_since_ap);
    //ROS_INFO("Angle diff: %f", angle_since_ap);
    //ROS_INFO("Angle trigger: %f", angleBetweenAnchorPoints);

    if(teachingStartTime != ros::Time(0))
    {	
        // Check if we traveled enough to get a new cloud.
        if(fabs(distance_since_ap) > distanceBetweenAnchorPoints ||
           fabs(angle_since_ap) > angleBetweenAnchorPoints)
        {
            ros::Time startTime = ros::Time::now();

            poseOfLastAnchor = lastOdomPosition;

            ROS_DEBUG("Saving a new anchor point");

            boost::thread cloudRecordingThread(recordCloud, *msg);

            ROS_DEBUG("The cloud callback took: %lf", (ros::Time::now() - startTime).toSec());
        }
        else
        {
            ROS_DEBUG("Got a cloud too close to the last anchor point. Ignored it.");
        }
    }
}

/*
void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[Y_BUTTON_INDEX] == 1 && teachingStartTime == ros::Time(0))
    {
        ROS_INFO("Starting Teaching.");
        teachingStartTime = ros::Time::now();
    }
}
*/

void keyboardCallback(const std_msgs::Bool msg)
{
	if(msg.data == 1 && teachingStartTime == ros::Time(0))
    {
        ROS_INFO("Starting Teaching.");
        teachingStartTime = ros::Time::now();
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Update the list of poses.
    lastOdomPosition = msg->pose.pose;
    lastYawRecorded = geo_util::quatTo2dYaw(msg->pose.pose.orientation);

    if(teachingStartTime != ros::Time(0))
    {
        *pPositionRecord <<
            boost::lexical_cast<std::string>((ros::Time::now() - teachingStartTime).toSec()) << "," <<
            geo_util::poseToString(lastOdomPosition);
    }
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(teachingStartTime != ros::Time(0))
    {
        *pSpeedRecord <<
            boost::lexical_cast<std::string>((ros::Time::now() - teachingStartTime).toSec()) << L_SEP << 
	    msg->linear.x << L_SEP << msg->angular.z << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_teach");
    ros::NodeHandle n("~");


    n.getParam(WORKING_DIRECTORY_PARAM, workingDirectory);
    n.param<double>(AP_TRIGGER_PARAM, 
            distanceBetweenAnchorPoints,
            DEFAULT_AP_TRIGGER);
    n.param<double>(ANGLE_AP_PARAM, angleBetweenAnchorPoints, DEFAULT_AP_ANGLE);

    if(chdir(workingDirectory.c_str()) != 0)
    {
        ROS_WARN("Could not switch to demanded directory. Using CWD instead.");
    };

    nextCloudIndex = 0;
    anchorPointList = std::vector<AnchorPoint>();

    ros::Subscriber sub = n.subscribe(POINT_CLOUD_TOPIC, 10, cloudCallback);
    //ros::Subscriber joystickTopic =
      //  n.subscribe(JOYSTICK_TOPIC, 5000, joystickCallback);
    ros::Subscriber keyboardTopic = n.subscribe(KEYBOARD_TOPIC, 5000, keyboardCallback);
    ros::Subscriber pose_topic =
        n.subscribe(POSE_ESTIMATE_TOPIC, 1000, odomCallback);
    ros::Subscriber velTopic =
        n.subscribe(VEL_TOPIC, 1000, velocityCallback);
    tf::TransformListener tfListener;
    ros::Publisher cloudRecorderTopic =
        n.advertise<husky_trainer::NamedPointCloud>(CLOUD_RECORDER_TOPIC, 100);
    pCloudRecorderTopic = &cloudRecorderTopic;

    std::ofstream positionRecord("positions.pl");
    std::ofstream speedRecord("speeds.sl");
    pPositionRecord = &positionRecord;
    pSpeedRecord = &speedRecord;

    // Fetch and store the transformation from the lidar to the base_link, we'll need it when
    // saving the point clouds.
    tLidarToBaseLink  =
            PointMatcher_ros::transformListenerToEigenMatrix<float>(tfListener, ROBOT_FRAME,
                                                                    LIDAR_FRAME, ros::Time(0));

    distanceTravelled = 0.0;
    lastYawRecorded = 0.0;
    teachingStartTime = ros::Time(0);

    ros::Rate loop_rate(LOOP_RATE);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    positionRecord.close();
    speedRecord.close();
    saveAnchorPointList(anchorPointList);


    ROS_INFO_STREAM("Recorded " << anchorPointList.size() << " anchor points.");

    return 0;
}
