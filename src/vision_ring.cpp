#include <vision_ring.h>

// 全局变量定义
int target_color = 0;
int mid_step = 0;
float land_target_y = 1;//默认黄色
float if_debug = 0;
float err_max = 0.2;

State mission_num = OFFBOARD;


void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "1.5: " << 1.5 << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
}




int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "template");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("image_raw", 10, color_seen);//摄像头
  ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, laser_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建服务客户端
  ros::ServiceClient param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

  nh.param<float>("err_max", err_max, 0.15);
  nh.param<float>("if_debug", if_debug, 1);
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
  if (choice != 1)return 0;
  ros::spinOnce();
  rate.sleep();
  
  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  //设置无人机的期望位置
 
  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // send a few setpoints before starting（维持offboard模式）
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout<<"ok"<<std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  param_set_client.waitForExistence();
  mavros_msgs::ParamSet vel_limit_cmd;
  vel_limit_cmd.request.param_id = "MPC_XY_VEL_MAX";
  vel_limit_cmd.request.value.real = 0.6; 
  if (param_set_client.call(vel_limit_cmd) && vel_limit_cmd.response.success) 
  {
    ROS_INFO(">>> SPEED LIMIT SET TO 0.6 M/S <<<");
  }

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();
  ros::Time last_request_arm = ros::Time::now();
  ros::Time last_request_mode = ros::Time::now();



  while (ros::ok())
  {
    switch (mission_num)
    {
      case OFFBOARD:

        // 1. 优先处理解锁 (Arming)
        if (!current_state.armed && (ros::Time::now() - last_request_arm > ros::Duration(3.0)))
        {
          if (arming_client.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("Vehicle armed");
          }
          last_request_arm = ros::Time::now();
        }

        // 2. 处理模式切换 (Offboard)
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request_mode > ros::Duration(3.0)))
        {
          if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
          {
            ROS_INFO("Offboard enabled");
          }
          last_request_mode = ros::Time::now();
        }
        

        if (current_state.armed && current_state.mode == "OFFBOARD")
        {
            ROS_INFO("State checked! Moving to TAKEOFF...");
            mission_num = TAKEOFF;
        }
        break;


      case TAKEOFF:
        // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
        {
          if (ros::Time::now() - last_request > ros::Duration(3.0))
          {
            mission_num = HOVER;
            ROS_INFO("进入任务模式");
            last_request = ros::Time::now();
            break;
          }
        }
        break;
      case HOVER:
        if (mission_pos_cruise(0, 0, 1.5, 0, err_max))
        {
          static bool hover_start = false;
          static ros::Time hover_time;

          if(!hover_start)
          {
            hover_time = ros::Time::now();
            ROS_INFO("开始悬停");
            hover_start = true;
          }


          if(ros::Time::now() - hover_time >= ros::Duration(10.0))
          {
            mission_num = AVOIDANCE;
            hover_start = false;
            ROS_INFO("已悬停10s");
            last_request = ros::Time::now();
          }
        }
        break;
      case AVOIDANCE:
        switch (mid_step)
          {
            case 0:
              if (mission_pos_cruise(2.3, 1.5, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 1:
              if (mission_pos_cruise(4.3, 1.5, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 2:
              if (mission_pos_cruise(6.5, 1.7, 1.5, 0.0 , err_max))
              {
                mission_num = THROUGH_DOOR;
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
          }
          apply_safety_corrections();
        break;
      case THROUGH_DOOR:
        switch (mid_step)
          {
            case 3:
              if (mission_pos_cruise(13, 1.7, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 4:
              if (mission_pos_cruise(13, 3.3, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 5:
              if (mission_pos_cruise(16, 3.3, 1.5, 0.0 , err_max))
              {
                mid_step++;
                mission_num = THROUGH_RING;
                last_request = ros::Time::now();
              }
              break;
          }
          apply_safety_corrections();
        break;
      case THROUGH_RING:
        switch (mid_step)
          {
            case 6:
              if (mission_pos_cruise(18.5, 2.35, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 7:
              if (mission_pos_cruise(19.5, 2.35, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 8:
              if (mission_pos_cruise(21.5,0.2, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 9:
              if (mission_pos_cruise(22.5, 0.2, 1.5, 0.0 , err_max))
              {
                mid_step++;
                mission_num = CRUISE;
                last_request = ros::Time::now();
              }
              break;
          }
          apply_safety_corrections();
        break;
      case CRUISE:
        switch (mid_step)
          {
            case 10:
              if (mission_pos_cruise(28, 3.2, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 11:
              if (mission_pos_cruise(28, 1, 1.5, 0.0 , err_max))
              {
                mid_step++;
                last_request = ros::Time::now();
              }
              break;
            case 12:
              if (mission_pos_cruise(28, -1.2, 1.5, 0.0 , err_max))
              {
                mid_step++;
                mission_num =LAND;
                last_request = ros::Time::now();
              }
              break;
          }
          apply_safety_corrections();
        break;
      case LAND:
        if(target_color == 1)
        {
          land_target_y = -1;
        }
        else if(target_color == 2)
        {
          land_target_y = 3;
        }
        switch (mid_step)
        {
          case 13:
            if (mission_pos_cruise(35, land_target_y, 1.5, 0.0 , err_max))
            {
              mid_step++;
              last_request = ros::Time::now();
            }
            break;
          case 14:
            if(precision_land())
            {
              mavros_msgs::SetMode land_mode;
              land_mode.request.custom_mode = "AUTO.LAND";
              
              if(set_mode_client.call(land_mode) && land_mode.response.mode_sent)
              {
                ROS_INFO("Vehicle LAND mode enabled!");
                mission_num = END; // 结束任务
                last_request = ros::Time::now();
              }
            }
            break;
          break;
        }
    }

    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
      
    if(mission_num == END) 
    {
      exit(0);
    }
  }
  return 0;
}