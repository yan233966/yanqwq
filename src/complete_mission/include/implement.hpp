#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <mavros_msgs/CommandLong.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <complete_mission/command.h>
#include <complete_mission/Position.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>

using namespace std;
enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Takeoff,
    Land,
    Arm,
    Disarm,
    Failsafe_land,
    Idle
};
#define ALTITUDE 0.8

mavros_msgs::PositionTarget setpoint_raw;
complete_mission::Position Position;
/************************************************************************
函数功能0-1：指令发出文件信息回调函数
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
complete_mission::command Command_Now;
complete_mission::command Command_Last;
void Command_cb(const complete_mission::command::ConstPtr &msg);
void Command_cb(const complete_mission::command::ConstPtr &msg)
{
    Command_Now = *msg;
}
/************************************************************************
函数功能1：无人机状态回调函数
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

/************************************************************************
函数功能2：回调函数接收无人机的里程计信息
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
    Position.position[0]=local_pos.pose.pose.position.x;
    Position.position[1]=local_pos.pose.pose.position.y;
    Position.position[2]=local_pos.pose.pose.position.z;
}

/************************************************************************
函数功能3：自主巡航，发布目标位置，控制无人机到达目标，采用local坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = init_position_x_take_off + x;
    setpoint_raw.position.y = init_position_y_take_off + y;
    setpoint_raw.position.z = init_position_z_take_off + z;
    setpoint_raw.yaw = yaw;
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max)
    {
        
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}
/************************************************************************
函数功能4：自主巡航，发布目标位置，控制无人机到达目标，采用机体坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
float current_position_cruise_last_position_yaw = 0;
bool current_position_cruise_flag = false;
bool current_position_cruise(float x, float y, float z, float yaw, float error_max);
bool current_position_cruise(float x, float y, float z, float yaw, float error_max)
{
    if (current_position_cruise_flag == false)
    {
        current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
        current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
        current_position_cruise_flag = true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = current_position_cruise_last_position_x + x;
    setpoint_raw.position.y = current_position_cruise_last_position_y + y;
    setpoint_raw.position.z = z;
    setpoint_raw.yaw = yaw;
    double current_yaw, a, b;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

    if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(current_yaw - yaw) < 0.1)
    {
        
        current_position_cruise_flag = false;
        return true;
    }
    return false;
}

