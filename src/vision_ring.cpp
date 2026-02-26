#include <vision_ring.h>

// 全局变量定义
int target_color = 0;
int mid_step = 0;
float land_target_y = 1; //默认黄色
float if_debug = 0;
float err_max = 0.2;

State mission_num = OFFBOARD;

// === 接收 YOLO 视觉数据的变量 ===
float yolo_u = 320.0;
float yolo_v = 240.0;
float yolo_w = 0.0;
ros::Time last_yolo_time;

void ring_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    yolo_u = msg->x;
    yolo_v = msg->y;
    yolo_w = msg->z; // 宽度藏在 Z 里
    last_yolo_time = ros::Time::now();
}

void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) std::cout << "自动offboard" << std::endl;
  else std::cout << "遥控器offboard" << std::endl;
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, ""); // 防止中文输出乱码

  ros::init(argc, argv, "template");
  ros::NodeHandle nh;

  // 订阅话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Subscriber ring_sub = nh.subscribe<geometry_msgs::Point>("/ring_center", 10, ring_cb); // 订阅 YOLO 穿环数据

  // 发布话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 客户端
  ros::ServiceClient param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Rate rate(20);

  nh.param<float>("err_max", err_max, 0.2);
  nh.param<float>("if_debug", if_debug, 0);
  print_param();
  
  int choice = if_debug;
  if (choice != 1) 
  {
    std::cout << "1 to go on , else to quit" << std::endl;
    std::cin >> choice;
  }
  else
  {
    std::cout << "已自动offboard" << std::endl;
  }
  if (choice != 1) return 0;
  
  ros::spinOnce();
  rate.sleep();
  
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
 
  setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "FCU Connected & Setpoints initialized" << std::endl;

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;



  ros::Time last_request_arm = ros::Time::now();
  ros::Time last_request_mode = ros::Time::now();
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    switch (mission_num)
    {
      case OFFBOARD:
        if (!current_state.armed && (ros::Time::now() - last_request_arm > ros::Duration(3.0)))
        {
          if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            ROS_INFO("Vehicle armed");
          last_request_arm = ros::Time::now();
        }
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request_mode > ros::Duration(3.0)))
        {
          if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            ROS_INFO("Offboard enabled");
          last_request_mode = ros::Time::now();
        }
        
        if (current_state.armed && current_state.mode == "OFFBOARD")
        {
            ROS_INFO("准备起飞...");
            mission_num = TAKEOFF;
        }
        break;

      case TAKEOFF:
        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
        {
          if (ros::Time::now() - last_request > ros::Duration(2.0))
          {
            mission_num = HOVER;
            ROS_INFO("到达起飞高度，准备巡航");
            last_request = ros::Time::now();
          }
        }
        break;

      case HOVER:
        // 简短悬停 3 秒，确保姿态稳定
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          if (ros::Time::now() - last_request > ros::Duration(3.0))
          {
            mission_num = ACHIEVE_START; // 进入航点任务
            mid_step = 0;
            ROS_INFO("开始航点飞行任务!");
          }
        }
        break;

      case ACHIEVE_START:
        switch (mid_step)
        {
          case 0:
            // 前往途经点1 (1, 0)，机头正向 X 轴
            if (mission_pos_cruise(1.0, 0.0, ALTITUDE, 0.0, err_max))
            {
              mid_step++;
              ROS_INFO("到达 (1,0)，转向前往 (1,4)");
            }
            break;
          case 1:
            // 前往途经点2 (1, 4)，此时在沿 Y 轴平移，机头转向 Y 轴正方向 (Yaw = 1.57)
            if (mission_pos_cruise(1.0, 4.0, ALTITUDE, 1.57, err_max))
            {
              mid_step++;
              ROS_INFO("到达 (1,4)，准备对准穿越门");
            }
            break;
          case 2:
            // 到达 (1, 4) 后，机头转回正前方 (Yaw = 0)，看向门 (2.5, 4)
            if (mission_pos_cruise(1.0, 4.0, ALTITUDE, 0.0, err_max))
            {
              mission_num = THROUGH_RING; // 机头对准后，进入穿环模式！
              ROS_INFO("机头对准完毕，启动 YOLO 视觉穿环伺服！");
            }
            break;
        }
        break;

      case THROUGH_RING:
        // 如果 YOLO 信号是新鲜的（0.5秒内），且识别到了门
        if (ros::Time::now() - last_yolo_time < ros::Duration(0.5) && yolo_w > 0)
        {
          mission_through_ring(yolo_u, yolo_v, yolo_w);
        }
        else
        {
          // 【彻底丢失视野保底】：调取全局记忆盲穿！
          setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512;
          setpoint_raw.coordinate_frame = 1;
          
          setpoint_raw.position.x = local_pos.pose.pose.position.x + 0.2; // 摸索前进
          
          // 如果之前成功估算过门的位置，直接调用
          if (has_gate_estimate) {
              setpoint_raw.position.y = estimated_gate_y;
              setpoint_raw.position.z = estimated_gate_z;
              // 顺便打印一下，看看它估算得准不准！
              ROS_INFO_THROTTLE(0.5, " 视野丢失，调用动态记忆航点 Y=%.2f, Z=%.2f", estimated_gate_y, estimated_gate_z);
          } else {
              // 极端情况：一眼都没看见过门，只能锁死当前航向盲飞
              setpoint_raw.position.y = local_pos.pose.pose.position.y;
              setpoint_raw.position.z = local_pos.pose.pose.position.z;
          }
          
          setpoint_raw.yaw = 0.0;
        }


        // 【退出条件】：门在 X=2.5。只要飞机的 X 坐标超过了 3.2，说明已经成功钻过门了！
        if (local_pos.pose.pose.position.x > 3.2)
        {
          ROS_INFO(" 成功穿过大门！前往终点...");
          mission_num = ACHIEVE_FINAL;
        }
        break;

      case ACHIEVE_FINAL:
        // 前往终点 (6, 4)
        if (mission_pos_cruise(6.0, 4.0, ALTITUDE, 0.0, err_max))
        {
          ROS_INFO("到达终点 (6,4)，准备降落");
          mission_num = LAND;
        }
        break;

      case LAND:
        if (precision_land())
        {
          mavros_msgs::SetMode land_mode;
          land_mode.request.custom_mode = "AUTO.LAND";
          if (set_mode_client.call(land_mode) && land_mode.response.mode_sent)
          {
            ROS_INFO("进入 AUTO.LAND 降落模式，任务圆满结束！");
            mission_num = END;
          }
        }
        break;
    }

    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
      
    if(mission_num == END) 
    {
      break; // 退出 while 循环，结束程序
    }
  }
  return 0;
}

