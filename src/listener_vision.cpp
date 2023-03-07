#include "ros/ros.h"
#include "ros/console.h"

#include "krssg_ssl_msgs/SSL_DetectionFrame.h"
#include "krssg_ssl_msgs/SSL_DetectionBall.h"
#include "krssg_ssl_msgs/SSL_DetectionRobot.h"

using namespace std;

typedef struct {
    float x;
    float y;
} BallPose;

typedef struct {
    int id;
    float x;
    float y;
    float orientation;
} RobotPose;

void getBallPosition(BallPose *ball_pose, vector<krssg_ssl_msgs::SSL_DetectionBall> ball) {
    ball_pose->x = ball[0].x;
    ball_pose->y = ball[0].y;
}

void getRobotPosition(RobotPose *robot_pose, vector<krssg_ssl_msgs::SSL_DetectionRobot> robots) {
    for (krssg_ssl_msgs::SSL_DetectionRobot robot : robots) {
        robot_pose[robot.robot_id].id = robot.robot_id;
        robot_pose[robot.robot_id].x = robot.x;
        robot_pose[robot.robot_id].y = robot.y;
        robot_pose[robot.robot_id].orientation = robot.orientation;
    }
}

void listenerCallback(const krssg_ssl_msgs::SSL_DetectionFrame& detection_frame) {

    // To get ball informations 
    vector<krssg_ssl_msgs::SSL_DetectionBall> detection_ball = detection_frame.balls;

    // To get robots informations of yellow team
    vector<krssg_ssl_msgs::SSL_DetectionRobot> detection_yellow_robots = detection_frame.robots_yellow;

    // To get robots informations of blue team
    vector<krssg_ssl_msgs::SSL_DetectionRobot> detection_blue_robots = detection_frame.robots_blue;

    // Declare structs to get ball and robots position
    BallPose ball_pose;
    RobotPose blue_poses[detection_yellow_robots.size()];
    RobotPose yellow_poses[detection_yellow_robots.size()];

    getBallPosition(&ball_pose, detection_ball);
    getRobotPosition(blue_poses, detection_blue_robots);
    getRobotPosition(yellow_poses, detection_yellow_robots);
} 

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/vision", 10000, listenerCallback);

    ros::spin();

    return 0;
}