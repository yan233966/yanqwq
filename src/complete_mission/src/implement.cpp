#include <implement.hpp>

int mission_num = 0;
int main(int argc, char **argv)
{
    // 防止中文输出乱码
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "complete_mission");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    // 创建一个Subscriber订阅者，订阅名为/mission/command的topic，注册回调函数Command_cb
    ros::Subscriber Command_sub = nh.subscribe<complete_mission::command>("/mission/command",10,Command_cb);
    // 发布无人机多维控制话题
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
    ros::Publisher position_sub = nh.advertise<complete_mission::Position>("/position",10);
    // 创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // 创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(20);

    // 等待连接到飞控
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // 设置无人机的期望位置
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    // 定义客户端变量，设置为offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 定义客户端变量，请求无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 记录当前时间，并赋值给变量last_request
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
        if (fabs(local_pos.pose.pose.position.z - 0.01 ) < 0.1)
        {
            if (ros::Time::now() - last_request > ros::Duration(3.0))
            {
                
                mission_num = 1;
                break;
            }
        }

        mission_pos_cruise(0, 0, 0.01, 0, 0.2);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
        
        
    }
    while (ros::ok())
    {
        if(Command_Last.command == Land)
        {
            Command_Now.command = Land;
        }

        switch (Command_Now.command)
        {
        
        case Move_ENU:
            mission_pos_cruise(Command_Now.pos_sp[0],Command_Now.pos_sp[1],Command_Now.pos_sp[2], 0, 0.1);
            break;
        case Hold:
            if (Command_Last.command != Hold)
            {
                current_position_cruise(0,0,0,0,0.1);
            }
            break;
        case Land:
            ROS_INFO("LAND");
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(offb_set_mode);
            mission_num = -1;
            break;
        }
        Command_Last = Command_Now;
        position_sub.publish(Position);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
