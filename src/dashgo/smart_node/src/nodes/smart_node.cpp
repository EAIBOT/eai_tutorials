/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointField.h"

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>


#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>

#include "smart_driver.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include "Console.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <math.h>
#include "smart_node.h"

#define DELAY_SECONDS 2


using namespace smart;
static bool flag = true;
static bool isPassed = true;
static bool isPassed_2 = true;
static bool is_recharge = false;
static SmartDriver smartdriver;
static bool isStoped = false;
static int sonar_safe_distance;
static bool useImu,useSonar;
static double max_vx;
static double max_vw;

static ros::Publisher *robot_cmd_vel_pub;

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )//限定x的上下限值然后输出
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))


const double ODOM_POSE_COVARIANCE[] = {1e-3, 0, 0, 0, 0, 0,
                                        0, 1e-3, 0, 0, 0, 0,
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0,
                                        0, 0, 0, 0, 1e6, 0,
                                        0, 0, 0, 0, 0, 1e3};
const double ODOM_POSE_COVARIANCE2[] = {1e-9, 0, 0, 0, 0, 0,
                                         0, 1e-3, 1e-9, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e-9};

const double ODOM_TWIST_COVARIANCE[] = {1e-3, 0, 0, 0, 0, 0,
                                         0, 1e-3, 0, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e3};
const double ODOM_TWIST_COVARIANCE2[] = {1e-9, 0, 0, 0, 0, 0,
                                          0, 1e-3, 1e-9, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e-9};




// std::vector<double> split(const std::string &s, char delim) {
//     std::vector<double> elems;
//     std::stringstream ss(s);
//     std::string number;
//     while(std::getline(ss, number, delim)) {
//         elems.push_back(atof(number.c_str()));
//     }
//     return elems;
// }

//速度发布回调函数
void cmdVelCallback(const geometry_msgs::Twist& msgIn){
    geometry_msgs::Twist robot_cmd_vel;
    double vx = msgIn.linear.x;
    double vw = msgIn.angular.z;

    uint8_t sonar[10];
    memcpy(sonar,smartdriver.get_pc_send_data()->sensor_data.sonar,sizeof(uint8_t)*10);
    if(smartdriver.get_pc_send_data()->charge_data.stop_status){
        robot_cmd_vel.linear.x = 0;
        robot_cmd_vel.linear.y = 0;
        robot_cmd_vel.angular.z = 0;
    }
    else{
        robot_cmd_vel.linear.x = msgIn.linear.x;
        robot_cmd_vel.linear.y = 0;
        robot_cmd_vel.angular.z = msgIn.angular.z;
    }
    robot_cmd_vel_pub->publish(robot_cmd_vel);
    if((isPassed == false || isPassed_2 == false) && vx > 0){
        vx = 0;
    }
    if(useSonar == true){
        //这里为紧急处理超声波测到的障碍物
        if(sonar[0] <= sonar_safe_distance && sonar[0] >= 2 && vx < 0){
            vx = 0;
            ROS_WARN("sonar0 smaller than safe_range[%d], cannot back",sonar_safe_distance);
        }
        if(sonar[1] <= sonar_safe_distance && sonar[1] >= 2 ){
            if(vx > 0){
                vx = 0;
            }

            if(vw < 0){
                vw = 0;
            }
            ROS_WARN("sonar1 smaller than safe_range[%d], only turn left",sonar_safe_distance);
        }
        if(sonar[2] <= sonar_safe_distance && sonar[2] >= 2 && vx > 0){
            vx = 0;
            ROS_WARN("sonar2 smaller than safe_range[%d], only back",sonar_safe_distance);
        }
        if(sonar[3] <= sonar_safe_distance && sonar[3] >= 2 ){
            if(vx > 0){
                vx = 0;
            }
            if(vw > 0){
                vw = 0;
            }
            ROS_WARN("sonar3 smaller than safe_range[%d], only turn right",sonar_safe_distance);
        }
        if(sonar[4] <= sonar_safe_distance && sonar[4] >= 2 && vx < 0){
            vx = 0;
            ROS_WARN("sonar4 smaller than safe_range[%d], cannot back",sonar_safe_distance);
        }
    }
    //to do:drop and dumper process

    //限速
    vx = LIMIT(vx,-max_vx,max_vx);
    vw = LIMIT(vw,-max_vw,max_vw);
//    ROS_WARN("VX=%.2f vw=%.2f\n\r",vx,vw);
    smartdriver.set_speed(vx,0,vw);
}

//摇杆速度发布回调函数
void HandlecmdVelCallback(const geometry_msgs::Twist& msgIn){
    
    double vx = msgIn.linear.x;
    double vw = msgIn.angular.z;

    //限速
    vx = LIMIT(vx,-max_vx,max_vx);
    vw = LIMIT(vw,-max_vw,max_vw);
//    ROS_WARN("VX=%.2f vw=%.2f\n\r",vx,vw);
    smartdriver.set_speed(vx,0,vw);
}

void isPassedCallback(const std_msgs::Int16 & msgIn){//这里要看一下是什么数据类型
    if(msgIn.data> 2){
        isPassed = false;
    }else{
        isPassed = true;
    }
}

void isPassedCallback_2(const std_msgs::Int16 & msgIn){//这里要看一下是什么数据类型
    if(msgIn.data > 2){
        isPassed_2 = false;
    }else{
        isPassed_2 = true;
    }
}

void handleRechargeCallback(const std_msgs::Int16& msgIn){//这里要看一下是什么数据类型
    if(msgIn.data == 1){
        smartdriver.set_automatic_recharge(1);
    }
    else if(msgIn.data == 0){
        smartdriver.set_automatic_recharge(0);
    }
}

void stop(void){
    isStoped = true;
    smartdriver.set_speed(0,0,0);
}

void resetEncoderCallback(const std_msgs::Bool& msgIn)
{
	if(msgIn.data == 1){
		smartdriver.ResetEncoder();
		
	}

}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "smart_node"); 

     printf("  _____ __  __          _____ _______  \n");  //如果需要打印'\ '则需要写出'\\'
     printf(" / ____|  \\/  |   /\\   |  __ \\__   __| \n");
     printf("| (___ | \\  / |  /  \\  | |__) | | |    \n");
     printf(" \\___ \\| |\\/| | / /\\ \\ |  _  /  | |    \n");
     printf(" ____) | |  | |/ ____ \\| | \\ \\  | |    \n");
     printf("|_____/|_|  |_/_/    \\_\\_|  \\_\\ |_|    \n");
     printf("\n");

     fflush(stdout);

    std::string port;
    int baudrate=115200;
    int data_update_rate = 0;
    int ros_publish_rate = 0;
    int driver_type = 0;
    std::string imu_frame_id;
    double sonar_height = 0.15;
    std::string config_path;

    result_t op_result;
    std::string base_frame("base_footprint");

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/port1");// /dev/ttyUSB0
    nh_private.param<int>("baudrate", baudrate, 115200); 
    nh_private.param<int>("data_update_rate", data_update_rate, 30);
    nh_private.param<int>("ros_publish_rate", ros_publish_rate, 50);
    nh_private.param<int>("driver_type", driver_type, 0);
    nh_private.param<std::string>("config_path", config_path, "/home/book/catkin_ws/src/smart_node/config/smart_config.yaml");
    nh_private.param<bool>("useImu", useImu, true);
    nh_private.param<bool>("useSonar", useSonar, true);
    nh_private.param<std::string>("imu_frame_id", imu_frame_id, "imu_base");
    nh_private.param<std::string>("base_frame", base_frame, "base_footprint");
    nh_private.param<double>("sonar_height", sonar_height, 0.15);
    nh_private.param<int>("sonar_safe_distance", sonar_safe_distance, 10);
    nh_private.param<double>("max_vx", max_vx, 1.0);
    nh_private.param<double>("max_vw", max_vw, 0.5);


    smartdriver.set_data_rate(data_update_rate);
    const pos_data_t *  encoder_pos = smartdriver.get_encoder_process_data(); //编码盘数据处理得到的坐标数据
    const send_pc_t* info = smartdriver.get_pc_send_data(); //所有数据的指针

    smart::console.message("Current SMART ROS SDK Version: %s\n",smartdriver.getSDKVersion().c_str());

    op_result = smartdriver.connect(port.c_str(), (uint32_t)baudrate,driver_type);
    if (op_result != RESULT_OK) {
        int seconds=0;
        while(seconds <= DELAY_SECONDS&&flag){
            sleep(2);
            seconds = seconds + 2;
            smartdriver.disconnect();
            op_result = smartdriver.connect(port.c_str(), (uint32_t)baudrate,driver_type);
            printf("[SMART INFO] Try to connect the port %s again  after %d s .\n", port.c_str() , seconds);
            if(op_result==RESULT_OK){
                break;
            }
        }
        
        if(seconds > DELAY_SECONDS){
            ROS_ERROR("SMART Cannot bind to the specified serial port %s" , port.c_str());
            smartdriver.disconnect();
            return -1;
        }
    }
   
    smart::console.show("Connected to SMART on port %s at %d \n" , port.c_str(), baudrate);
    op_result = smartdriver.smart_init();
    if (op_result != RESULT_OK) {
        smart::console.error("SMART driver init failed!\n");
        return -1;
    }
    smart::console.show("SMART driver init successful! \n");

    //将读取的参数写入配置文件。注意，此操作会清空文件然后重新写入
    std::string productnum(smartdriver.get_pc_send_data()->config_param.base_conf.productNum);
    std::ofstream fout(config_path.c_str());
    YAML::Emitter out(fout);
    out << YAML::BeginMap;
    out << YAML::Key << "ProductNUM";
    out << YAML::Value << productnum.c_str();
    out << YAML::Key << "Version";
    out << YAML::Value << smartdriver.getVersion()->version_info;
    out << YAML::Key << "base_type";
    out << YAML::Value << (int)(smartdriver.get_pc_send_data()->config_param.base_conf.base_type);
    out << YAML::Comment("CIRCLE = 0 SQUARE = 1");
    out << YAML::Key << "base_radius_mm";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_radius_mm;
    out << YAML::Comment("the radius of platform");
    out << YAML::Key << "distance_between_wheels_mm";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.distance_between_wheels_mm;
    out << YAML::Key << "chassis_pos0_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.chassis_pos[0].x_to_center_mm;
    out << YAML::Key << "chassis_pos0_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.chassis_pos[0].y_to_center_mm;
    out << YAML::Key << "chassis_pos1_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.chassis_pos[1].x_to_center_mm;
    out << YAML::Key << "chassis_pos1_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.chassis_pos[1].y_to_center_mm;
    out << YAML::Key << "chassis_pos2_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.chassis_pos[2].x_to_center_mm;
    out << YAML::Key << "chassis_pos2_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.chassis_pos[2].y_to_center_mm;
    out << YAML::Key << "chassis_pos3_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.chassis_pos[3].x_to_center_mm;
    out << YAML::Key << "chassis_pos3_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.chassis_pos[3].y_to_center_mm;
    //超声波尺寸
    out << YAML::Key << "base_snoar_num";
    out << YAML::Value << (int)(smartdriver.get_pc_send_data()->config_param.base_conf.base_snoar_num);
    out << YAML::Key << "base_snoars0_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[0].x_to_center_mm;
    out << YAML::Key << "base_snoars0_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[0].y_to_center_mm;
    out << YAML::Key << "base_snoars0_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[0].z_to_center_mm;
    out << YAML::Key << "base_snoars0_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[0].anti_clockwise_angle_to_center_degree;
    out << YAML::Comment("anti clockwise angle to center degree");

    out << YAML::Key << "base_snoars1_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[1].x_to_center_mm;
    out << YAML::Key << "base_snoars1_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[1].y_to_center_mm;
    out << YAML::Key << "base_snoars1_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[1].z_to_center_mm;
    out << YAML::Key << "base_snoars1_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[1].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_snoars2_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[2].x_to_center_mm;
    out << YAML::Key << "base_snoars2_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[2].y_to_center_mm;
    out << YAML::Key << "base_snoars2_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[2].z_to_center_mm;
    out << YAML::Key << "base_snoars2_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[2].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_snoars3_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[3].x_to_center_mm;
    out << YAML::Key << "base_snoars3_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[3].y_to_center_mm;
    out << YAML::Key << "base_snoars3_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[3].z_to_center_mm;
    out << YAML::Key << "base_snoars3_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[3].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_snoars4_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[4].x_to_center_mm;
    out << YAML::Key << "base_snoars4_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[4].y_to_center_mm;
    out << YAML::Key << "base_snoars4_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[4].z_to_center_mm;
    out << YAML::Key << "base_snoars4_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[4].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_snoars5_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[5].x_to_center_mm;
    out << YAML::Key << "base_snoars5_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[5].y_to_center_mm;
    out << YAML::Key << "base_snoars5_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[5].z_to_center_mm;
    out << YAML::Key << "base_snoars5_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[5].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_snoars6_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[6].x_to_center_mm;
    out << YAML::Key << "base_snoars6_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[6].y_to_center_mm;
    out << YAML::Key << "base_snoars6_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[6].z_to_center_mm;
    out << YAML::Key << "base_snoars6_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[6].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_snoars7_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[7].x_to_center_mm;
    out << YAML::Key << "base_snoars7_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[7].y_to_center_mm;
    out << YAML::Key << "base_snoars7_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[7].z_to_center_mm;
    out << YAML::Key << "base_snoars7_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[7].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_snoars8_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[8].x_to_center_mm;
    out << YAML::Key << "base_snoars8_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[8].y_to_center_mm;
    out << YAML::Key << "base_snoars8_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[8].z_to_center_mm;
    out << YAML::Key << "base_snoars8_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[8].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_snoars9_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[9].x_to_center_mm;
    out << YAML::Key << "base_snoars9_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[9].y_to_center_mm;
    out << YAML::Key << "base_snoars9_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[9].z_to_center_mm;
    out << YAML::Key << "base_snoars9_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_snoars[9].anti_clockwise_angle_to_center_degree;

    //碰撞尺寸
    out << YAML::Key << "base_bumper_num";
    out << YAML::Value << (int)(smartdriver.get_pc_send_data()->config_param.base_conf.base_bumper_num);

    out << YAML::Key << "base_bumpers0_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[0].x_to_center_mm;
    out << YAML::Key << "base_bumpers0_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[0].y_to_center_mm;
    out << YAML::Key << "base_bumpers0_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[0].z_to_center_mm;
    out << YAML::Key << "base_bumpers0_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[0].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_bumpers1_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[1].x_to_center_mm;
    out << YAML::Key << "base_bumpers1_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[1].y_to_center_mm;
    out << YAML::Key << "base_bumpers1_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[1].z_to_center_mm;
    out << YAML::Key << "base_bumpers1_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[1].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_bumpers2_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[2].x_to_center_mm;
    out << YAML::Key << "base_bumpers2_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[2].y_to_center_mm;
    out << YAML::Key << "base_bumpers2_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[2].z_to_center_mm;
    out << YAML::Key << "base_bumpers2_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[2].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_bumpers3_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[3].x_to_center_mm;
    out << YAML::Key << "base_bumpers3_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[3].y_to_center_mm;
    out << YAML::Key << "base_bumpers3_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[3].z_to_center_mm;
    out << YAML::Key << "base_bumpers3_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_bumpers[3].anti_clockwise_angle_to_center_degree;

    //跌落尺寸
    out << YAML::Key << "base_drop_num";
    out << YAML::Value << (int)(smartdriver.get_pc_send_data()->config_param.base_conf.base_drop_num);

    out << YAML::Key << "base_drops0_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[0].x_to_center_mm;
    out << YAML::Key << "base_drops0_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[0].y_to_center_mm;
    out << YAML::Key << "base_drops0_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[0].z_to_center_mm;
    out << YAML::Key << "base_drops0_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[0].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_drops1_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[1].x_to_center_mm;
    out << YAML::Key << "base_drops1_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[1].y_to_center_mm;
    out << YAML::Key << "base_drops1_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[1].z_to_center_mm;
    out << YAML::Key << "base_drops1_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[1].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_drops2_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[2].x_to_center_mm;
    out << YAML::Key << "base_drops2_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[2].y_to_center_mm;
    out << YAML::Key << "base_drops2_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[2].z_to_center_mm;
    out << YAML::Key << "base_drops2_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[2].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_drops3_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[3].x_to_center_mm;
    out << YAML::Key << "base_drops3_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[3].y_to_center_mm;
    out << YAML::Key << "base_drops3_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[3].z_to_center_mm;
    out << YAML::Key << "base_drops3_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_drops[3].anti_clockwise_angle_to_center_degree;

    //雷达尺寸
    out << YAML::Key << "base_lidar_num";
    out << YAML::Value << (int)(smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar_num);

    out << YAML::Key << "base_lidar0_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar[0].x_to_center_mm;
    out << YAML::Key << "base_lidar0_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar[0].y_to_center_mm;
    out << YAML::Key << "base_lidar0_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar[0].z_to_center_mm;
    out << YAML::Key << "base_lidar0_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar[0].anti_clockwise_angle_to_center_degree;

    out << YAML::Key << "base_lidar1_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar[1].x_to_center_mm;
    out << YAML::Key << "base_lidar1_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar[1].y_to_center_mm;
    out << YAML::Key << "base_lidar1_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar[1].z_to_center_mm;
    out << YAML::Key << "base_lidar1_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_lidar[1].anti_clockwise_angle_to_center_degree;

    //摄像头尺寸
    out << YAML::Key << "base_camera_x";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_camera.x_to_center_mm;
    out << YAML::Key << "base_camera_y";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_camera.y_to_center_mm;
    out << YAML::Key << "base_camera_z";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_camera.z_to_center_mm;
    out << YAML::Key << "base_camera_w";
    out << YAML::Value << smartdriver.get_pc_send_data()->config_param.base_conf.base_camera.anti_clockwise_angle_to_center_degree;

    out << YAML::EndMap;
    fout.close();//不关闭文件，数据不会写入

    sleep(1);//延时一段时间等待文件写入完成。

    ros::NodeHandle nh;

    // Subscriptions
//    ros::Subscriber sub_vel = nh.subscribe( "cmd_vel" ,1000,&cmdVelCallback);
    ros::Subscriber sub_vel = nh.subscribe("smoother_cmd_vel" ,1000,&cmdVelCallback);
    ros::Subscriber handlecmd_sub_vel = nh.subscribe("handle_cmd_vel" ,1000,&HandlecmdVelCallback);
    robot_cmd_vel_pub = new ros::Publisher(nh.advertise<geometry_msgs::Twist >("robot_cmd_vel",5));
    ros::Subscriber sub_passed1 = nh.subscribe("is_passed" ,1000,&isPassedCallback);
    ros::Subscriber sub_passed2 = nh.subscribe("is_passed_2" ,1000,&isPassedCallback_2);

    // sonar publisher
    ros::Publisher sonar0_pub = nh.advertise<sensor_msgs::Range >("sonar0",5);
    ros::Publisher sonar1_pub = nh.advertise<sensor_msgs::Range >("sonar1",5);
    ros::Publisher sonar2_pub = nh.advertise<sensor_msgs::Range >("sonar2",5);
    ros::Publisher sonar3_pub = nh.advertise<sensor_msgs::Range >("sonar3",5);
    ros::Publisher sonar4_pub = nh.advertise<sensor_msgs::Range >("sonar4",5);
    ros::Publisher sonar5_pub = nh.advertise<sensor_msgs::Range >("sonar5",5);
    ros::Publisher sonar6_pub = nh.advertise<sensor_msgs::Range >("sonar6",5);
    ros::Publisher sonar7_pub = nh.advertise<sensor_msgs::Range >("sonar7",5);
    ros::Publisher sonar8_pub = nh.advertise<sensor_msgs::Range >("sonar8",5);
    ros::Publisher sonar9_pub = nh.advertise<sensor_msgs::Range >("sonar9",5);

    ros::Publisher sonar_pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2> ("sonar_cloudpoint2", 5);
    ros::Publisher sonar_pub_cloud = nh.advertise<sensor_msgs::PointCloud>("sonar_cloudpoint",5);
    double sonar_maxval = 3.5;
    double sonar_cloud[5][3]={{100.0,0.105,0.1},{100.0,-0.105,0.1},{0.2,100.0,0.1},{0.2,-100.0,0.1},{-100.0,0.0,0.1}};

    sonar_cloud[0][0] = info->config_param.base_conf.base_snoars[0].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[0].anti_clockwise_angle_to_center_degree));
    sonar_cloud[0][1] = info->config_param.base_conf.base_snoars[0].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[0].anti_clockwise_angle_to_center_degree));
    sonar_cloud[0][2] = sonar_height;

    sonar_cloud[1][0] = info->config_param.base_conf.base_snoars[1].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[1].anti_clockwise_angle_to_center_degree));
    sonar_cloud[1][1] = info->config_param.base_conf.base_snoars[1].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[1].anti_clockwise_angle_to_center_degree));
    sonar_cloud[1][2] = sonar_height;

    sonar_cloud[2][0] = info->config_param.base_conf.base_snoars[2].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[2].anti_clockwise_angle_to_center_degree));
    sonar_cloud[2][1] = info->config_param.base_conf.base_snoars[2].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[2].anti_clockwise_angle_to_center_degree));
    sonar_cloud[2][2] = sonar_height;

    sonar_cloud[3][0] = info->config_param.base_conf.base_snoars[3].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[3].anti_clockwise_angle_to_center_degree));
    sonar_cloud[3][1] = info->config_param.base_conf.base_snoars[3].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[3].anti_clockwise_angle_to_center_degree));
    sonar_cloud[3][2] = sonar_height;

    sonar_cloud[4][0] = info->config_param.base_conf.base_snoars[4].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[4].anti_clockwise_angle_to_center_degree));
    sonar_cloud[4][1] = info->config_param.base_conf.base_snoars[4].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[4].anti_clockwise_angle_to_center_degree));
    sonar_cloud[4][2] = sonar_height;

    //imu publisher
    ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu >("imu",5);
    ros::Publisher imuAnglePub = nh.advertise<std_msgs::Float32>("imu_angle",5);

    // Set up the odometry broadcaster
    ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry >("odom",5);
    tf::TransformBroadcaster odomBroadcaster;

    ros::Publisher lEncoderPub = nh.advertise<std_msgs::Int32>("Lencoder",5);
    ros::Publisher rEncoderPub = nh.advertise<std_msgs::Int32>("Rencoder",5);

    ros::Publisher bat_voltage_pub = nh.advertise<std_msgs::Int32>("voltage_value",5);
    ros::Publisher charge_voltage_pub = nh.advertise<std_msgs::Int32>("charge_voltage_value",5);
    ros::Publisher bat_voltage_percentage_pub = nh.advertise<std_msgs::Int32>("voltage_percentage",5);

    ros::Publisher emergencybt_pub = nh.advertise<std_msgs::Int16>("emergencybt_status",5);

    ros::Subscriber sub_recharge = nh.subscribe("recharge_handle" ,1000,&handleRechargeCallback);
    ros::Publisher recharge_pub = nh.advertise<std_msgs::Int16>("recharge_status",5);
    ros::Publisher recharge_way_pub = nh.advertise<std_msgs::Int32>("recharge_way",5);

    //drop publisher
    ros::Publisher drop_pub = nh.advertise<std_msgs::UInt8>("drop",5);
//    ros::Publisher drop1_pub = nh.advertise<uint8_t>("drop1",5);
//    ros::Publisher drop2_pub = nh.advertise<uint8_t>("drop2",5);
//    ros::Publisher drop3_pub = nh.advertise<uint8_t>("drop3",5);

    //bumper publisher
    ros::Publisher bumper_pub = nh.advertise<std_msgs::UInt8>("bumpers",5);
//    ros::Publisher bumper1_pub = nh.advertise<uint8_t>("bumper1",5);
//    ros::Publisher bumper2_pub = nh.advertise<uint8_t>("bumper2",5);
//    ros::Publisher bumper3_pub = nh.advertise<uint8_t>("bumper3",5);


    // nh_private.param<bool>("angle_fixed", angle_fixed, "true");
    // nh_private.param<bool>("resolution_fixed", resolution_fixed, "true");
    // nh_private.param<bool>("heartbeat", heartbeat, "false");
    // nh_private.param<bool>("low_exposure", low_exposure, "false");
    // nh_private.param<double>("angle_max", angle_max , 180);
    // nh_private.param<double>("angle_min", angle_min , -180);
    // nh_private.param<int>("samp_rate", samp_rate, 4);
    // nh_private.param<double>("range_max", max_range , 16.0);
    // nh_private.param<double>("range_min", min_range , 0.08);
    // nh_private.param<double>("frequency", _frequency , 7.0);
    // nh_private.param<std::string>("ignore_array",list,"");

	//message for resetting encoder
	ros::Subscriber sub_reset_encoder = nh.subscribe("clear_encoder" ,1,&resetEncoderCallback);




    ros::Rate rate(ros_publish_rate);


    while (ros::ok()) {
        try{

            // start_scan_time = ros::Time::now();

            // end_scan_time = ros::Time::now();      
             
            // start_scan_time.sec = start_time/1000000000ul;
            // start_scan_time.nsec = start_time%1000000000ul;
            // end_scan_time.sec = end_time/1000000000ul;
            // end_scan_time.nsec = end_time%1000000000ul;
            // scan_duration = (end_scan_time - start_scan_time).toSec();
            //发布encoder数据
            std_msgs::Int32 lencoder,rencoder;
            lencoder.data = smartdriver.get_pc_send_data()->encoder_data.lencoder;
            lEncoderPub.publish(lencoder);
            rencoder.data = info->encoder_data.rencoder;
            rEncoderPub.publish(rencoder);
            //发布电源数据
            std_msgs::Int32 vol;
            vol.data = info->charge_data.bat_vol;
            bat_voltage_pub.publish(vol);
            vol.data = info->charge_data.charge_vol;
            charge_voltage_pub.publish(vol);
            vol.data = info->charge_data.battary_percent;
            bat_voltage_percentage_pub.publish(vol);
            vol.data = info->charge_data.charge_type;
            recharge_way_pub.publish(vol);

            std_msgs::Int16 int16_data;

            int16_data.data = info->charge_data.stop_status;
            emergencybt_pub.publish(int16_data);
            int16_data.data = info->charge_data.recharge_status;
            recharge_pub.publish(int16_data);
            std_msgs::UInt8 uint8_data;
            //发布传感器数据
            uint8_data.data = info->sensor_data.drop;
            drop_pub.publish(uint8_data);
            uint8_data.data = info->sensor_data.bumper;
            bumper_pub.publish(uint8_data);
            //发布超声波数据
            if(useSonar == true)
            {
                sensor_msgs::Range sonar0_range;
                sonar0_range.header.stamp = ros::Time::now();
                sonar0_range.header.frame_id = "/sonar0";
                sonar0_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
                sonar0_range.field_of_view = 0.3f;
                sonar0_range.min_range = 0.04f;
                sonar0_range.max_range = 0.8f;
                sonar0_range.range = info->sensor_data.sonar[0]/100.0f;
                if(sonar0_range.range == 0.0f){
                    sonar0_range.range = 1.0;
                }
                else if(sonar0_range.range > sonar0_range.max_range){
                    sonar0_range.range = sonar0_range.max_range;
                }
                sonar0_pub.publish(sonar0_range);
                if(sonar0_range.range >= 0.5f || sonar0_range.range == 0.0f){
                    sonar_cloud[0][0] = info->config_param.base_conf.base_snoars[0].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[0].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[0][1] = info->config_param.base_conf.base_snoars[0].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[0].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[0][2] = sonar_height;
                }
                else{
                    sonar_cloud[0][0] = info->config_param.base_conf.base_snoars[0].x_to_center_mm*0.001f + sonar0_range.range * cos(DEG2RAD(info->config_param.base_conf.base_snoars[0].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[0][1] = info->config_param.base_conf.base_snoars[0].y_to_center_mm*0.001f + sonar0_range.range * sin(DEG2RAD(info->config_param.base_conf.base_snoars[0].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[0][2] = sonar_height;
                }

                sensor_msgs::Range sonar1_range;
                sonar1_range.header.stamp = ros::Time::now();
                sonar1_range.header.frame_id = "/sonar1";
                sonar1_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
                sonar1_range.field_of_view = 0.3f;
                sonar1_range.min_range = 0.04f;
                sonar1_range.max_range = 0.8f;
                sonar1_range.range = info->sensor_data.sonar[1]/100.0f;
                if(sonar1_range.range == 0.0f){
                    sonar1_range.range = 1.0f;
                }
                else if(sonar1_range.range > sonar1_range.max_range){
                    sonar1_range.range = sonar1_range.max_range;
                }
                sonar1_pub.publish(sonar1_range);
                if(sonar1_range.range >= 0.5f || sonar1_range.range == 0.0f){
                    sonar_cloud[1][0] = info->config_param.base_conf.base_snoars[1].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[1].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[1][1] = info->config_param.base_conf.base_snoars[1].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[1].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[1][2] = sonar_height;
                }
                else{
                    sonar_cloud[1][0] = info->config_param.base_conf.base_snoars[1].x_to_center_mm*0.001f + sonar1_range.range * cos(DEG2RAD(info->config_param.base_conf.base_snoars[1].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[1][1] = info->config_param.base_conf.base_snoars[1].y_to_center_mm*0.001f + sonar1_range.range * sin(DEG2RAD(info->config_param.base_conf.base_snoars[1].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[1][2] = sonar_height;
                }

                sensor_msgs::Range sonar2_range;
                sonar2_range.header.stamp = ros::Time::now();
                sonar2_range.header.frame_id = "/sonar2";
                sonar2_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
                sonar2_range.field_of_view = 0.3f;
                sonar2_range.min_range = 0.04f;
                sonar2_range.max_range = 0.8f;
                sonar2_range.range = info->sensor_data.sonar[2]/100.0f;
                if(sonar2_range.range == 0.0f){
                    sonar2_range.range = 1.0f;
                }
                else if(sonar2_range.range > sonar2_range.max_range){
                    sonar2_range.range = sonar2_range.max_range;
                }
                sonar2_pub.publish(sonar2_range);
                if(sonar2_range.range >= 0.5f || sonar2_range.range == 0.0f){
                    sonar_cloud[2][0] = info->config_param.base_conf.base_snoars[2].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[2].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[2][1] = info->config_param.base_conf.base_snoars[2].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[2].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[2][2] = sonar_height;
                }
                else{
                    sonar_cloud[2][0] = info->config_param.base_conf.base_snoars[2].x_to_center_mm*0.001f + sonar2_range.range * cos(DEG2RAD(info->config_param.base_conf.base_snoars[2].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[2][1] = info->config_param.base_conf.base_snoars[2].y_to_center_mm*0.001f + sonar2_range.range * sin(DEG2RAD(info->config_param.base_conf.base_snoars[2].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[2][2] = sonar_height;
                }

                sensor_msgs::Range sonar3_range;
                sonar3_range.header.stamp = ros::Time::now();
                sonar3_range.header.frame_id = "/sonar3";
                sonar3_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
                sonar3_range.field_of_view = 0.3f;
                sonar3_range.min_range = 0.04f;
                sonar3_range.max_range = 0.8f;
                sonar3_range.range = info->sensor_data.sonar[3]/100.0f;
                if(sonar3_range.range == 0.0f){
                    sonar3_range.range = 1.0f;
                }
                else if(sonar3_range.range > sonar3_range.max_range){
                    sonar3_range.range = sonar3_range.max_range;
                }
                sonar3_pub.publish(sonar3_range);
                if(sonar3_range.range >= 0.5f || sonar3_range.range == 0.0f){
                    sonar_cloud[3][0] = info->config_param.base_conf.base_snoars[3].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[3].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[3][1] = info->config_param.base_conf.base_snoars[3].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[3].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[3][2] = sonar_height;
                }
                else{
                    sonar_cloud[3][0] = info->config_param.base_conf.base_snoars[3].x_to_center_mm*0.001f + sonar3_range.range * cos(DEG2RAD(info->config_param.base_conf.base_snoars[3].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[3][1] = info->config_param.base_conf.base_snoars[3].y_to_center_mm*0.001f + sonar3_range.range * sin(DEG2RAD(info->config_param.base_conf.base_snoars[3].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[3][2] = sonar_height;
                }

                sensor_msgs::Range sonar4_range;
                sonar4_range.header.stamp = ros::Time::now();
                sonar4_range.header.frame_id = "/sonar4";
                sonar4_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
                sonar4_range.field_of_view = 0.3f;
                sonar4_range.min_range = 0.04f;
                sonar4_range.max_range = 0.8f;
                sonar4_range.range = info->sensor_data.sonar[4]/100.0f;
                if(sonar4_range.range == 0.0f){
                    sonar4_range.range = 1.0f;
                }
                else if(sonar4_range.range > sonar4_range.max_range){
                    sonar4_range.range = sonar4_range.max_range;
                }
                sonar4_pub.publish(sonar4_range);
                if(sonar4_range.range >= 0.5f || sonar4_range.range == 0.0f){
                    sonar_cloud[4][0] = info->config_param.base_conf.base_snoars[4].x_to_center_mm*0.001f + sonar_maxval * cos(DEG2RAD(info->config_param.base_conf.base_snoars[4].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[4][1] = info->config_param.base_conf.base_snoars[4].y_to_center_mm*0.001f + sonar_maxval * sin(DEG2RAD(info->config_param.base_conf.base_snoars[4].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[4][2] = sonar_height;
                }
                else{
                    sonar_cloud[4][0] = info->config_param.base_conf.base_snoars[4].x_to_center_mm*0.001f + sonar4_range.range * cos(DEG2RAD(info->config_param.base_conf.base_snoars[4].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[4][1] = info->config_param.base_conf.base_snoars[4].y_to_center_mm*0.001f + sonar4_range.range * sin(DEG2RAD(info->config_param.base_conf.base_snoars[4].anti_clockwise_angle_to_center_degree));
                    sonar_cloud[4][2] = sonar_height;
                }


                pcl::PointCloud<pcl::PointXYZ> cloud;
                sensor_msgs::PointCloud2 output;

                // Fill in the cloud data
                cloud.width  = 5;
                cloud.height = 1;
                cloud.points.resize(cloud.width * cloud.height);

    //            sensor_msgs::PointCloud2 pcloud;
                sensor_msgs::PointCloud pcloud;
                geometry_msgs::Point32 sonarPoint;
                for(uint8_t inc=0;inc < 5;inc++ ){
                    sonarPoint.x = sonar_cloud[inc][0];
                    sonarPoint.y = sonar_cloud[inc][1];
                    sonarPoint.z = sonar_cloud[inc][2];
                    pcloud.points.push_back(sonarPoint);


                    cloud.points[inc].x = sonar_cloud[inc][0];
                    cloud.points[inc].y = sonar_cloud[inc][1];
                    cloud.points[inc].z = sonar_cloud[inc][2];
                }
                pcloud.header.stamp = ros::Time::now();
                pcloud.header.frame_id="/base_footprint";
                //pcloud = pcl::PCLPointCloud2::create_cloud_xyz32(pcloud.header, sonar_cloud);

                sonar_pub_cloud.publish(pcloud);

                //Convert the cloud to ROS message
                pcl::toROSMsg(cloud, output);
                output.header.frame_id = "base_footprint";

                sonar_pub_cloud2.publish(output);
            }

            //发布imu数据
            if(useImu == true){
                const imu_data_t* imu = &info->imu_data;
                float yaw = imu->yaw/100.0f;
                float roll = imu->roll/100.0f;
                float pitch = imu->pitch/100.0f;
                float yaw_rate = imu->yaw_rate/100.0f;
                float roll_rate = imu->roll_rate/100.0f;
                float pitch_rate = imu->pitch_rate/100.0f;
                sensor_msgs::Imu imu_data;
                imu_data.header.stamp = ros::Time::now();
                imu_data.header.frame_id = imu_frame_id;
                imu_data.orientation_covariance[0] = 1000000;
                imu_data.orientation_covariance[1] = 0;
                imu_data.orientation_covariance[2] = 0;
                imu_data.orientation_covariance[3] = 0;
                imu_data.orientation_covariance[4] = 1000000;
                imu_data.orientation_covariance[5] = 0;
                imu_data.orientation_covariance[6] = 0;
                imu_data.orientation_covariance[7] = 0;
                imu_data.orientation_covariance[8] = 0.000001;
                geometry_msgs::Quaternion imu_quaternion;

                imu_quaternion.x = 0.0;
                imu_quaternion.y = 0.0;
                imu_quaternion.z = sin(-1*yaw*3.1416/(180 *2.0));
                imu_quaternion.w = cos(-1*yaw*3.1416/(180 *2.0));
                imu_data.orientation = imu_quaternion;

                imu_data.linear_acceleration.x = imu->xacc;
                imu_data.linear_acceleration.y = imu->yacc;
                imu_data.linear_acceleration.z = imu->zacc;
                imu_data.linear_acceleration_covariance[0] = -1;

                imu_data.angular_velocity_covariance[0] = -1;
                imu_data.angular_velocity.x = DEG2RAD(roll_rate);
                imu_data.angular_velocity.y = DEG2RAD(pitch_rate);
                imu_data.angular_velocity.z = DEG2RAD(yaw_rate);

                imuPub.publish(imu_data);
                std_msgs::Float32 imuAngle;
                imuAngle.data = -1*DEG2RAD(yaw);
                imuAnglePub.publish(imuAngle);
            }

            ros::Time current_time = ros::Time::now();
            geometry_msgs::Quaternion odom_quaternion;
//            geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(encoder_pos->pos_dethe);
            odom_quaternion.x = 0.0;
            odom_quaternion.y = 0.0;
            odom_quaternion.z = sin(encoder_pos->pos_dethe / 2.0);
            odom_quaternion.w = cos(encoder_pos->pos_dethe / 2.0);

            if(useImu == false){
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = base_frame;

                odom_trans.transform.translation.x = encoder_pos->pos_x;
                odom_trans.transform.translation.y = encoder_pos->pos_y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quaternion;

                //send the transform
                odomBroadcaster.sendTransform(odom_trans);

            }

            //发布odom数据
            nav_msgs::Odometry odom;
            odom.header.frame_id = "odom";
            odom.child_frame_id = base_frame;
            odom.header.stamp = current_time;
            odom.pose.pose.position.x = encoder_pos->pos_x;
            odom.pose.pose.position.y = encoder_pos->pos_y;
            odom.pose.pose.position.z = 0;
            odom.pose.pose.orientation = odom_quaternion;

            odom.twist.twist.linear.x = encoder_pos->pos_vx;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = encoder_pos->pos_vw;

//            odom.pose.covariance = 0;//ODOM_POSE_COVARIANCE;
//            odom.twist.covariance = ODOM_TWIST_COVARIANCE;
            uint8_t i=0;
            if(encoder_pos->pos_vx == 0 && encoder_pos->pos_vw == 0){ //底盘不动时用下面这组协方差
                for(i=0;i<_countof(ODOM_POSE_COVARIANCE);i++){
                   odom.pose.covariance[i] = ODOM_POSE_COVARIANCE2[i];
                }
                for(i=0;i<_countof(ODOM_TWIST_COVARIANCE);i++){
                   odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE2[i];
                }
            }
            else{
                for(i=0;i<_countof(ODOM_POSE_COVARIANCE);i++){
                   odom.pose.covariance[i] = ODOM_POSE_COVARIANCE[i];
                }
                for(i=0;i<_countof(ODOM_TWIST_COVARIANCE);i++){
                   odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE[i];
                }
            }


            odomPub.publish(odom);







            rate.sleep();
            ros::spinOnce();
        }catch(std::exception &e){//
                ROS_ERROR_STREAM("Unhandled Exception: " << e.what() );
                break;
        }catch(...){//another exception
                ROS_ERROR("Unhandled Exception:Unknown ");
                break;
        }
    }

    smartdriver.disconnect();
    delete robot_cmd_vel_pub;
    printf("[SMART INFO] Now SMART is disconnecting .......\n");

    return 0;
}
