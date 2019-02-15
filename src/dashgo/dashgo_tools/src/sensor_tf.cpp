#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point32.h"
#include <math.h>

float x = 0.0;
float y = 0.0;
float qx = 0.0;
float qy = 0.0;
float qz = 1.0;
float qw = 1.0;
bool flag = false;
ros::Publisher obstacle_pub;

void robot_poseCallback(const geometry_msgs::Pose msg)
{
        if(!flag)
            return;
        x = msg.position.x;
        y = msg.position.y;
        qx = msg.orientation.x;
        qy = msg.orientation.y;
        qz = msg.orientation.z;
        qw = msg.orientation.w;
        flag = false;
}


void topic_callback(const sensor_msgs::PointCloud msg)
{
        flag = true;
        float angle = atan2(2.0*(qx*qy+qz*qw), 1.0-2.0*(qy*qy+qz*qz));
        sensor_msgs::PointCloud obstacle;
        ros::Time now = ros::Time::now();
        //obstacle.header.stamp.toSec=now.toSec();
        //obstacle.header.stamp.nsecs=now.toNsecs;
        obstacle.header.frame_id=msg.header.frame_id;
        int j = sizeof(msg.points)/sizeof(msg.points[0]);
        //int i=0;
        for(int i=0;i<j;i++)
        {
            geometry_msgs::Point32 point;
            point.x=x - msg.points[i].y * sin(angle) + msg.points[i].x * cos(angle);
            point.y=y + msg.points[i].x * sin(angle) + msg.points[i].y * cos(angle);
            point.z=msg.points[i].z;
            obstacle.points.push_back(point);
        }
        obstacle_pub.publish(obstacle);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ObstacleTF");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("robot_pose", 1, robot_poseCallback);
    ros::Subscriber sub2 = n.subscribe("other_sensor_data", 1, topic_callback);
    obstacle_pub = n.advertise<sensor_msgs::PointCloud>("other_sensor_data_tf", 1);
    ros::Rate loop_rate(10);
    //while (ros::ok()){}
    ros::spin();
    return 0;
}
