 #include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <complete_mission/command.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
complete_mission::command Command_now;

float radius_circle; 		     //圆形半径
float height_circle;                //飞行高度
float sleep_time;

float circle_sleep_time;
int number_points;		     //割圆边数 定点数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);
    ros::Publisher move_pub = nh.advertise<complete_mission::command>("/mission/command", 10);//发布移动指令
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("radius_circle", radius_circle, 0.5);
    nh.param<float>("height_circle", height_circle, 0.5);
    nh.param<float>("sleep_time", sleep_time, 10.0);
    nh.param<int>("number_points",number_points, 36);
    nh.param<float>("circle_sleep_time",circle_sleep_time, 0.1);
    ros::Rate circle_rate(1.0/circle_sleep_time);
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;
    if(check_flag != 1) return -1;
    int i=0;
    int comid = 0;
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //takeoff
    i = 0;
    while (i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_circle;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 0----->takeoff"<<endl;
        i++;
    }
    
    //point left bottom
    i = 0;
    while(i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = -radius_circle;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_circle;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 1----->left-bottom"<<endl;
        i++;
    }
    //point a circle
    for(int num_points=0 ;num_points<=number_points;num_points++)
    {
        float angle = M_PI + num_points * (2 * M_PI / number_points);    
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = radius_circle * cos(angle);
        Command_now.pos_sp[1] = radius_circle * sin(angle);
        Command_now.pos_sp[2] = height_circle;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        for(int k=0;k<=2;k++)
        {
            move_pub.publish(Command_now);
            circle_rate.sleep();
        }
        if (num_points>=32)
        {
            move_pub.publish(Command_now);
            circle_rate.sleep();
        }
        cout << "Point "<< num_points<<endl;
        
    }
    //point return
    i = 0;
    while(i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_circle;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 37----->return"<<endl;
        i++;
    }

    //降落
    Command_now.command = Land;
    while (ros::ok())
    {
      move_pub.publish(Command_now);
      rate.sleep();
      cout << "Land"<<endl;
    }
    rate.sleep();
    cout << "Mission complete, exiting...."<<endl;
    return 0;
}

