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
#include <sensor_msgs/PointCloud2.h>

using namespace std;

#define ALTITUDE 1.2f

enum State 
{
    OFFBOARD,
    TAKEOFF,          // 起飞
    HOVER,            // 悬停
    ACHIEVE_START,    // 开始位置
    THROUGH_RING,     // 穿环动作
    ACHIEVE_FINAL,    // 结束位置
    LAND,             // 降落
    END               // 结束
};

static float estimated_gate_y = 0.0;
static float estimated_gate_z = 0.0;
static bool has_gate_estimate = false;

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
函数 5: 视觉伺服穿环控制 - 【动态坐标估计记忆】+【盲区记忆重放】
*************************************************************************/
bool mission_through_ring(float u, float v, float w);
bool mission_through_ring(float u, float v, float w)
{
    // === 1. 相机内参 ===
    const float focal_length = 343.5; 
    const float img_cx = 320.0;       
    const float img_cy = 240.0;       
    const float W_real = 0.9;         

    float safe_w = (w > 10.0) ? w : 10.0; 
    float dist_x = (focal_length * W_real) / safe_w; 
    
    // 计算相机坐标系下的 Y(左右) 和 Z(上下) 偏差
    float dist_y = -((u - img_cx) * dist_x) / focal_length; 
    float dist_z = -((v - img_cy) * dist_x) / focal_length; 

    // ===  核心逻辑：动态估计门的世界坐标 (远距离时触发)  ===
    // 只有在距离适中 (1.0m~5.0m)，且框比较完整时，我们才信任视觉并更新记忆
    if (dist_x > 1.0 && dist_x < 5.0 && safe_w < 400.0) 
    {
        // 结合无人机当前的世界坐标和偏航角，算出大门当前的绝对物理坐标！
        float current_gate_y = local_pos.pose.pose.position.y + dist_x * sin(yaw) + dist_y * cos(yaw);
        float current_gate_z = local_pos.pose.pose.position.z + dist_z;

        if (!has_gate_estimate) {
            // 第一次看到门，直接写入记忆
            estimated_gate_y = current_gate_y;
            estimated_gate_z = current_gate_z;
            has_gate_estimate = true;
        } else {
            // 指数移动平均 (EMA) 滤波：平滑跳变，新数据占 10%，历史记忆占 90%
            estimated_gate_y = 0.1 * current_gate_y + 0.9 * estimated_gate_y;
            estimated_gate_z = 0.1 * current_gate_z + 0.9 * estimated_gate_z;
        }
    }

    // === 2. 近距离盲穿保护 (重放记忆) ===
    if (safe_w > 500.0 || dist_x < 0.7) 
    {
        setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512;
        setpoint_raw.coordinate_frame = 1;
        
        float step_forward = 0.5; // 加速冲刺
        setpoint_raw.position.x = local_pos.pose.pose.position.x + step_forward * cos(yaw);
        
        if (has_gate_estimate) {
            //  提取记忆：使用最后一次完美的估计位置进行穿刺！
            setpoint_raw.position.y = estimated_gate_y;
            setpoint_raw.position.z = estimated_gate_z;
            ROS_INFO_THROTTLE(0.2, " 盲穿: 调用记忆坐标 Y=%.2f, Z=%.2f", estimated_gate_y, estimated_gate_z);
        } else {
            // 如果毫无记忆，只能保持当前高度平飞
            setpoint_raw.position.y = local_pos.pose.pose.position.y + step_forward * sin(yaw);
            setpoint_raw.position.z = local_pos.pose.pose.position.z;
        }
        
        setpoint_raw.yaw = yaw;
        return true;
    }

    // === 3. 正常视觉伺服：向记忆中心点靠拢 ===
    float Kp_y = 0.6;  
    float Kp_z = 1.0;  
    float step_forward = 0.2; 
    
    float align_error = sqrt(dist_y * dist_y + dist_z * dist_z);

    // 刹车对准逻辑
    if (align_error > 0.15 && dist_x > 0.8) {
        step_forward = 0.05; 
    }

    if (dist_x > 5.0) dist_x = 5.0; 

    float delta_X = step_forward * cos(yaw) - (dist_y * Kp_y) * sin(yaw);
    float delta_Y = step_forward * sin(yaw) + (dist_y * Kp_y) * cos(yaw);

    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512;
    setpoint_raw.coordinate_frame = 1;

    setpoint_raw.position.x = local_pos.pose.pose.position.x + delta_X;
    setpoint_raw.position.y = local_pos.pose.pose.position.y + delta_Y;
    setpoint_raw.position.z = local_pos.pose.pose.position.z + (dist_z * Kp_z);
    
    setpoint_raw.yaw = yaw;

    return true;
}