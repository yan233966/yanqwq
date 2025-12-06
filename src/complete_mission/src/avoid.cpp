#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <algorithm>
#include <complete_mission/command.h>
#include <complete_mission/Position.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

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
struct Avoidance_Params
{
   float safe_distance;
   float warning_distance;
   float max_avoid_angle;
   float step_size;
   float max_avoid_distance;
   float r;
};
struct Way
{
   float x,y,z;
   float attraction;
};

class Avoidance_Position
{
private:
    static const int MAX_PATH_SIZE=100;
    Avoidance_Params params_;
    Way full_path_[MAX_PATH_SIZE];
    int path_size_;
    Way current_goal_;
    Way current_position_;
    bool obstacle_detection_;
    int current_goal_k_;
    
public:
  Avoidance_Position():
    path_size_(0),
    obstacle_detection_(false),
    current_goal_k_(0)
  {
    params_.safe_distance=1.5;
    params_.warning_distance=1.0;
    params_.max_avoid_angle =M_PI / 3;
    params_.step_size = 1.0;
    params_.max_avoid_distance = 1.0;
    params_.r = 2.0;
    
    memset(full_path_,0,sizeof(full_path_));
    current_goal_={0,0,0,0};
    current_position_={0,0,0,0};
  }
  void setParams(const Avoidance_Params& params) {
        params_ = params;
    }
  void setFullPath(const Way points[],int count)
  {
    path_size_ = (count>MAX_PATH_SIZE) ? MAX_PATH_SIZE : count;
    
    for(int i = 0; i < path_size_;i++)
    {
        full_path_[i]=points[i];
    }
    
    if (path_size_ > 0)
    {
        current_goal_ = full_path_[0];
        current_goal_k_ = 0;
    }
    
  }
  void selectNext()
  {
      if(path_size_ > 0)
      {
          if(current_goal_k_ == path_size_ - 1 )
          {
              return;
          }
          current_goal_k_=(current_goal_k_+1) % path_size_;
          current_goal_ = full_path_[current_goal_k_];
          
      }
  }
  
  bool isPathEmpty() const
  {
      return path_size_ == 0;
  }
  
  int getPathSize()const
  {
      return path_size_;
  }
  void updatePosition(float x, float y,float z)
  {
      current_position_.x = x;
      current_position_.y = y;
      current_position_.z = z;
  }
  float getCurrentAttraction() const
  {
      return current_goal_.attraction;
  }
  Way calculateAvoidance(const sensor_msgs::LaserScan& lidar)
  {
      Way avoidance_command = current_goal_;
      obstacle_detection_ = false;
      
      float dx = current_goal_.x - current_position_.x;
      float dy = current_goal_.y - current_position_.y;
      float target_distance = sqrt(dx*dx + dy*dy);
      cout<<"target_distance"<<target_distance<<endl;
      float target_angle = atan2(dy, dx);
      cout<<"target_angle"<<target_angle<<endl;
      float current_attraction = getCurrentAttraction();
      cout << "current_attraction: " << current_attraction << endl;
      
      if (target_distance < 0.1)
      {
          selectNext();
          ROS_INFO("Reached target point, cruise point task completed.");
          return avoidance_command;
      }
      
      float best_angle = target_angle;
      float min_cost = FLT_MAX; // FLT_MAX 是 float 类型的最大正值
      float search_angle_range = params_.max_avoid_angle / max(1.0f, current_attraction);
      for(float angle = target_angle -search_angle_range;
       angle<=target_angle + search_angle_range;
       angle += 0.1)
      {
          float cost = evaluateDirection(angle, lidar, target_angle);
          
          if(cost < min_cost)
          {
              min_cost = cost;
              best_angle = angle;
          }
      }
      float move_distance = min(params_.step_size, target_distance);
      cout<<"move_distance"<<move_distance<<endl;
      avoidance_command.x = current_position_.x + cos(best_angle) * move_distance;
      avoidance_command.y = current_position_.y + sin(best_angle) * move_distance;
      avoidance_command.z = current_goal_.z;
      
      if(min_cost > 0.1) // 如果最小代价较大，说明有障碍物
      {
          obstacle_detection_ = true;
      }
      cout<<"min_cost "<<min_cost<<endl;
      return avoidance_command;
      
  }
private:
  float evaluateDirection(float angle, const sensor_msgs::LaserScan& lidar, float target_angle)
  {
      float cost=0.0;
      float dx = current_goal_.x - current_position_.x;
      float dy = current_goal_.y - current_position_.y;
      float target_distance = sqrt(dx*dx + dy*dy);
      float move_distance = min(params_.step_size, target_distance);
      
      Way future_position;
      future_position.x = current_position_.x + cos(angle) * move_distance;
      future_position.y = current_position_.y + sin(angle) * move_distance;
      future_position.z = current_position_.z;
      
      
      float obstacle_cost = calculate_cost_obstacle(angle, lidar,future_position);
      cost+= obstacle_cost * 15.0;
      
      float cost_direction = fabs(angle - target_angle);
      cost += cost_direction * 2.0;
      return cost;
  }
  float calculate_cost_obstacle(float angle, const sensor_msgs::LaserScan& lidar, const Way& future_position)
  {
      float min_future_distance = lidar.range_max;
      float width = 0.3;
      
      // 计算未来位置与各个障碍物的距离
      for(int i = 0; i < lidar.ranges.size(); i++)
      {
          if(lidar.ranges[i] > lidar.range_min && lidar.ranges[i] < lidar.range_max)
          {
              // 将激光雷达数据转换为ENU坐标系中的障碍物位置
              float obstacle_angle = lidar.angle_min + i * lidar.angle_increment;
              float obstacle_x = current_position_.x + lidar.ranges[i] * cos(obstacle_angle);
              float obstacle_y = current_position_.y + lidar.ranges[i] * sin(obstacle_angle);
              
              // 计算未来位置与障碍物的距离
              float dx_future = future_position.x - obstacle_x;
              float dy_future = future_position.y - obstacle_y;
              float future_obstacle_distance = sqrt(dx_future*dx_future + dy_future*dy_future);
              
              if(future_obstacle_distance < min_future_distance)
              {
                  min_future_distance = future_obstacle_distance;
              }
          }
      }
      
      // 基于未来位置与障碍物的距离计算代价
      if(min_future_distance < params_.warning_distance)
      {
          return 100.0;
      }
      else if(min_future_distance < params_.safe_distance)
      {
          return (params_.safe_distance - min_future_distance) * params_.r;
      }
      return 0.0;
  }  
};
/*class ThroughRingPosition
{
    private:
    bool judge_ring_;
    Way central_position_;
    static constexpr float distance_ring = 2.0;
    Way current_position_;
    Way ring_point_;
    Way ring_position[2];
    std::vector<float> obstacle_y;
    std::vector<float> obstacle_x;
    std::vector<bool> point_valid;
    public:
     ThroughRingPosition():judge_ring_(false)
     {
     ring_position[0] = {0,0,0};
     ring_position[1] = {0,0,0};
     current_position_={0,0,0};
     central_position_={0,0,0};
     ring_point_={0,0,0};
     }
      bool JudgeRing(const sensor_msgs::LaserScan &lidar)
      {
          judge_ring_ = false;
          
          if(lidar.ranges.empty()) 
          {
                return false;
          }
          int total_points = lidar.ranges.size();
          const int START = total_points * 7/16;  // 前方区域的起始索引
          const int END = total_points * 9/16;    // 前方区域的结束索引
          const float LENGTH_HOLD = 2.0;
          const float MIN_GAP_WIDTH = 0.3;
          const float MAX_GAP_WIDTH = 2.0;  
          const int MIN_POINTS_SIDE = 2;
          int gap_start = -1, gap_end = -1;
          float gap_width = 0.0;
          for(int i = START; i < END - 1; i++)
          {
              float range1 = lidar.ranges[i];
              float range2 = lidar.ranges[i+1];
              if(std::isinf(range1) || std::isnan(range1) || std::isinf(range2) || std::isnan(range2)) 
              {
                  continue;
              }
              float angle1 = lidar.angle_min + i * lidar.angle_increment;
              float angle2 = lidar.angle_min + (i+1) * lidar.angle_increment;
              float x1 = current_position_.x + range1 * cos(angle1);
              float y1 = current_position_.y + range1 * sin(angle1);
              float x2 = current_position_.x + range2 * cos(angle2);
              float y2 = current_position_.y + range2 * sin(angle2);
              float point_distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
              
              if(point_distance > LENGTH_HOLD) 
              {
                  if(gap_start == -1) 
                  {
                      gap_start = i;  // 记录缺口开始位置
                  }
                  gap_end = i+1;      // 更新缺口结束位置
              }
          }
          
          if(gap_start == -1 || gap_end == -1) 
          {
              return false;  // 没找到缺口
          }
          gap_width = (gap_end - gap_start) * lidar.angle_increment;
          
          if(gap_width < MIN_GAP_WIDTH || gap_width > MAX_GAP_WIDTH)
          {
              return false;
          }
          int left_side_points = 0;
          int right_side_points = 0;
          
          for(int i = gap_start - 5; i < gap_start; i++) 
          {
              if(i >= 0 && i < total_points) 
              {
                  float range = lidar.ranges[i];
                  if(!std::isinf(range) && !std::isnan(range)) 
                  {
                      left_side_points++;
                  }
              }
          }
          
          for(int i = gap_end; i < gap_end + 5; i++) 
          {
              if(i >= 0 && i < total_points) 
              {
                  float range = lidar.ranges[i];
                  if(!std::isinf(range) && !std::isnan(range)) 
                  {
                      right_side_points++;
                  }
              }
          }
          if(left_side_points < MIN_POINTS_SIDE || right_side_points < MIN_POINTS_SIDE) 
          {
              return false;  // 缺口两侧点数不足
          }
          int left = gap_start;
          int right = gap_end;
          
          float left_angle = lidar.angle_min + left * lidar.angle_increment;
          float left_range = lidar.ranges[left];
          float left_x = current_position_.x + left_range * cos(left_angle);
          float left_y = current_position_.y + left_range * sin(left_angle);
          float right_angle = lidar.angle_min + right * lidar.angle_increment;
          float right_range = lidar.ranges[right];
          float right_x = current_position_.x + right_range * cos(right_angle);
          float right_y = current_position_.y + right_range * sin(right_angle);
          
          
          central_position_.x = (left_x + right_x) / 2.0;
          central_position_.y = (left_y + right_y) / 2.0;
          central_position_.z = 1.2;
          
          judge_ring_ = true;
          cout << "[RingDebug] 检测到环！" << endl;
        cout << "  缺口宽度: " << gap_width << " 弧度 (" << gap_width*180.0/M_PI << " 度)" << endl;
        cout << "  环中心: (" << central_position_.x << ", " << central_position_.y << ")" << endl;
      
        cout << "  左侧点数: " << left_side_points << ", 右侧点数: " << right_side_points << endl;
          return judge_ring_;
      }
      void RingPosition()
      {
         
         ring_position[0].x = central_position_.x;
         ring_position[0].y = central_position_.y-distance_ring;
         ring_position[0].z = 1.2; 
         ring_position[1].x = central_position_.x;
         ring_position[1].y = central_position_.y+distance_ring;
         ring_position[1].z = 1.2; 
      }
      Way GetPoint()
      {
          ring_point_.x = ring_position[0].x;
          ring_point_.y = ring_position[0].y;
          ring_point_.z = ring_position[0].z;
          if (fabs(ring_point_.x-current_position_.x)<0.1 && fabs(ring_point_.y-current_position_.y)<0.1)
          {
              ring_point_.x = ring_position[1].x;
              ring_point_.y = ring_position[1].y;
              ring_point_.z = ring_position[1].z;
          }
          return ring_point_;
      }
      bool JudgeOver()
      {
          if (fabs(ring_position[1].x-current_position_.x)<0.1 && fabs(ring_position[1].y-current_position_.y)<0.1)
          {
              judge_ring_=false;
          }
          return judge_ring_;
      }
      void updatePosition(float x, float y,float z)
      {
         current_position_.x = x;
         current_position_.y = y;
         current_position_.z = z;
      }
        
};*/
//ThroughRingPosition through_ring_controller;
Avoidance_Position avoidance_controller;
complete_mission::command Command_now;
sensor_msgs::LaserScan Lidar_Now;
complete_mission::Position Position;
void Lidar_cb(const sensor_msgs::LaserScan::ConstPtr &msg);
void Lidar_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    Lidar_Now = *msg;
}
void Position_cb(const complete_mission::Position::ConstPtr &msg)
{
    Position = *msg; 
    avoidance_controller.updatePosition(
    Position.position[0],
    Position.position[1],
    Position.position[2]
    );
    /*through_ring_controller.updatePosition(
    Position.position[0],
    Position.position[1],
    Position.position[2]
    );*/
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoid");
    ros::NodeHandle nh("~");
    ros::Rate rate(10.0);
    ros::Publisher move_pub = nh.advertise<complete_mission::command>("/mission/command", 10);
    ros::Subscriber avoid_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 50, Lidar_cb);
    ros::Subscriber position_sub = nh.subscribe<complete_mission::Position>("/position", 10, Position_cb);
    Way path[] = {{8,2,1.5,1.0},{17,2.4,1.5,1.0},{19,2.4,1.5,4.8},{20,0.2,1.5,1.0},{22,0.2,1.5,4.2},{35,1,1.5,1.0}};
    avoidance_controller.setFullPath(path,6);
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    Avoidance_Params params;
    nh.param<float>("safe_distance", params.safe_distance, 1.1);
    nh.param<float>("warning_distance", params.warning_distance, 0.6);
    nh.param<float>("max_avoid_angle", params.max_avoid_angle, M_PI/3);
    nh.param<float>("step_size", params.step_size, 1.0);
    nh.param<float>("max_avoid_distance",params.max_avoid_distance, 2.0);
    nh.param<float>("r",params.r, 2.0);
    avoidance_controller.setParams(params);
    
    
    
    int comid = 0;
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // 输入1,继续，其他，退出程序
    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;
    if(check_flag != 1) return -1;
    //takeoff
    while(fabs(Position.position[0])>0.1 || fabs(Position.position[1])>0.1 || fabs(Position.position[2] - 1.5)>0.1)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = 1.5;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid++;
        move_pub.publish(Command_now);
        
        ros::spinOnce();
        rate.sleep();
    }
    // 起飞完成后，悬停10秒
    ros::Time start_time = ros::Time::now();
    while (ros::Time::now() - start_time < ros::Duration(10.0))
    {
        // 持续发送当前位置指令以保持悬停
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = 1.5;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid++;
        move_pub.publish(Command_now);
    
        ros::spinOnce();
        rate.sleep();
    }

    cout << "Hovered for 10 seconds" << endl;
    float x,y,z;
     while (ros::ok()) {
         //bool ring_detected=through_ring_controller.JudgeRing(Lidar_Now);
         //bool ring_passed = through_ring_controller.JudgeOver();
         Way next_point = avoidance_controller.calculateAvoidance(Lidar_Now);;
         /*if(ring_detected && !ring_passed)
         {
            through_ring_controller.RingPosition();
            next_point = through_ring_controller.GetPoint();
         }*/
         cout<<next_point.x<<endl;
         Command_now.command = Move_ENU;
         Command_now.sub_mode = 0;
         Command_now.pos_sp[0] = next_point.x;
         Command_now.pos_sp[1] = next_point.y;
         Command_now.pos_sp[2] = next_point.z;
         Command_now.yaw_sp = 0;
         Command_now.comid = comid++;
         cout<<"next_point.x"<<endl;
         cout<<"position.x"<<Position.position[0]<<endl;
         if(fabs(Position.position[0]-35.0)<0.1 && fabs(Position.position[1]-1)<0.1 )
         {
             static const ros::Time start_time_land = ros::Time::now();
             if(ros::Time::now() - start_time_land > ros::Duration(3.0))
             {
                 Command_now.command = Land;
                 cout<<"Land"<<endl;
             }
         }
         
         move_pub.publish(Command_now);
         ros::spinOnce();
         rate.sleep();
          
    }
    
    return 0;
}

