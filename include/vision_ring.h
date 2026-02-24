#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

#define ALTITUDE 1.2f

enum State 
{
    OFFBOARD,
    TAKEOFF,          // 起飞
    HOVER,            // 悬停
    AVOIDANCE,        // 避障
    THROUGH_DOOR,     // 穿门动作
    THROUGH_RING,     // 穿环动作
    CRUISE,           // 巡航
    LAND,             // 降落
    END               // 结束
};

float dist_front = 10.0;
float dist_left = 10.0;
float dist_right = 10.0;
float dist_back = 10.0;

extern int target_color;
extern State mission_num;

mavros_msgs::PositionTarget setpoint_raw;

/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
从里程计信息中提取无人机的位置信息和姿态信息
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
		ROS_INFO("初始位置记录完成: 高度=%.3f米", init_position_z_take_off);
	}
}


/************************************************************************
函数 3: 无人机位置控制
控制无人机飞向（x, y, z）位置，target_yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
	if (mission_pos_cruise_flag == false)
	{
		mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
		mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
		mission_pos_cruise_flag = true;
	}
	// 忽略速度和加速度控制，只控制位置(Position)和偏航(Yaw)
	setpoint_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
							 mavros_msgs::PositionTarget::IGNORE_VY |
							 mavros_msgs::PositionTarget::IGNORE_VZ |
							 mavros_msgs::PositionTarget::IGNORE_AFX |
							 mavros_msgs::PositionTarget::IGNORE_AFY |
							 mavros_msgs::PositionTarget::IGNORE_AFZ |
							 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = target_yaw;
	//ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, target_yaw * 180.0 / M_PI, x + init_position_x_take_off, y + init_position_y_take_off, z + init_position_z_take_off, target_yaw * 180.0 / M_PI );
	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}

/************************************************************************
函数 4:降落
无人机当前位置作为降落点，缓慢下降至地面
返回值为bool型，表示是否降落完成
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land();
bool precision_land()
{
	if (!precision_land_init_position_flag)
	{
		precision_land_init_position_x = local_pos.pose.pose.position.x;
		precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
		precision_land_init_position_flag = true;
	}
	setpoint_raw.position.x = precision_land_init_position_x;
	setpoint_raw.position.y = precision_land_init_position_y;
	setpoint_raw.position.z = -0.15;
	setpoint_raw.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
    if(ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}

/************************************************************************
函数 5:颜色识别
检测图像中心颜色以判断色块颜色
*************************************************************************/
void color_seen(const sensor_msgs::Image::ConstPtr &msg)
{
	if (mission_num > 1||target_color != 0) return;

	int center_x = msg->width / 2;
    int center_y = msg->height / 2;
    int index = (center_y * msg->width + center_x) * 3;

	int r = msg->data[index];
	int g = msg->data[index + 1];
	int b = msg->data[index + 2];

	if (r > 100 && r > g * 1.5 && r > b * 1.5) 
    {
        ROS_INFO(">>> Detect Color: RED! (R=%d G=%d B=%d)", r, g, b);
		target_color = 1;
    }
    else if (b > 100 && b > r * 1.5 && b > g * 1.5) 
    {
        ROS_INFO(">>> Detect Color: BLUE! (R=%d G=%d B=%d)", r, g, b);
		target_color = 2;
    }
}

/************************************************************************
函数 6:位置微调
根据雷达数据微调目标点
*************************************************************************/
void laser_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int len = msg->ranges.size();
    float min_f = 10.0, min_l = 10.0, min_r = 10.0, min_b = 10.0;
    
    // 假设 0=前, len/4=左, len/2=后, 3*len/4=右
    
    // 1. 前方 (Front): -30 ~ +30 度
    int range_f = len / 6; 
    for(int i=0; i<range_f/2; i++) if(msg->ranges[i]>0.1 && msg->ranges[i]<min_f) min_f = msg->ranges[i];
    for(int i=len-range_f/2; i<len; i++) if(msg->ranges[i]>0.1 && msg->ranges[i]<min_f) min_f = msg->ranges[i];

    // 2. 左侧 (Left): +30 ~ +150 度 (覆盖整个左半边)
    int idx_30 = len / 12;
    int idx_150 = len * 5 / 12;
    for(int i=idx_30; i<idx_150; i++) {
        if(msg->ranges[i] > 0.1 && msg->ranges[i] < min_l) min_l = msg->ranges[i];
    }

    // 3. 右侧 (Right): +210 ~ +330 度 (覆盖整个右半边)
    int idx_210 = len * 7 / 12;
    int idx_330 = len * 11 / 12;
    for(int i=idx_210; i<idx_330; i++) {
        if(msg->ranges[i] > 0.1 && msg->ranges[i] < min_r) min_r = msg->ranges[i];
    }

    dist_front = min_f;
    dist_left = min_l;
    dist_right = min_r;
}
// 安全修正函数
void apply_safety_corrections()
{
    // === 参数设置 ===
    float safe_margin = 0.6;  // 触发避障的距离
    float stop_dist = 0.3;    // 前方急停距离
    
    // 强力推移距离 
    float push_force = 0.2;   

	bool is_ring_mode = (mission_num == THROUGH_RING); 

    // === 1. 穿环专用：强力居中 (Lidar) ===
    // 只有当左右都有数据（看到了两边的柱子）才启用
    if (is_ring_mode && dist_left < 0.5 && dist_right < 0.5)
    {
        // 居中误差
        float center_err = dist_left - dist_right;
        
        // 目标：死死咬住中心线
        float corr_y = center_err * 0.5; 

        // 限幅：虽然系数大，但单次修正量不要太大，防止甩尾
        if (corr_y > 0.2) corr_y = -0.2;
        if (corr_y < -0.2) corr_y = 0.2;

        setpoint_raw.position.y += corr_y;
        
        // ROS_INFO_THROTTLE(0.2, "Ring Centering: L=%.2f R=%.2f Err=%.2f", dist_left, dist_right, center_err);
        return; // 穿环时优先执行这个，跳过后面的普通避障
    }
    bool is_dangerous = false; // 标记是否处于危险状态

    // 1. 左右避障 (基于当前位置的强力弹开)
    if (dist_left < safe_margin) 
    {
        is_dangerous = true;
        
        //放弃原目标
        setpoint_raw.position.y = local_pos.pose.pose.position.y + push_force;
    }
    else if (dist_right < safe_margin) 
    {
        is_dangerous = true;
        
        setpoint_raw.position.y = local_pos.pose.pose.position.y - push_force;
    }

    // 2. 自动居中 (在不危险但需要微调时使用)
    // 只有在左右都安全，但通道较窄时才启用这个温和的逻辑
    else if (dist_left < 1.5 && dist_right < 1.5)
    {
        float center_err = dist_left - dist_right;
        if (fabs(center_err) > 0.2) // 只有偏差较大才修，防止抖动
        {
            // 在原目标基础上微调 (因为还要继续赶路)
            // 这里系数给大一点，因为还要对抗向前的速度矢量
            setpoint_raw.position.y += (center_err * 0.3); 
        }
    }

    // 3. 前方防撞与速度压制
    // 如果处于避障状态 (is_dangerous)，或者前方有障碍，限制前进速度
    if (is_dangerous || dist_front < 1.5)
    {
        // 限制目标点 X 轴只能在当前位置前方很近的地方 (例如 0.2米)
        // 这样飞控计算出的前进速度就会很慢，给侧向避障留出时间
        float max_forward_step = 0.2; 
        
        // 原计划的 X 目标
        float target_x = setpoint_raw.position.x;
        // 当前 X
        float current_x = local_pos.pose.pose.position.x;

        // 如果原目标太远，强制拉回来
        if (target_x - current_x > max_forward_step) {
            setpoint_raw.position.x = current_x + max_forward_step;
        }
    }

    // 4. 前方急停 (最高优先级)
    if (dist_front < stop_dist)
    {
        ROS_WARN_THROTTLE(0.5, "Emergency Stop!");
        setpoint_raw.position.x = local_pos.pose.pose.position.x; // 锁死 X
        // Y 轴保持上面的避障逻辑，允许它左右躲避
    }
}