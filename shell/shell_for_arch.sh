#!/bin/bash

# =========================================================================
#  [配置区] 根据项目需求修改变量
# =========================================================================

# 1. 路径配置
WS_PATH="/workspace/project_ws"               # 应用层工作空间
LIB_PATH="/workspace/catkin_ws"               # 底层库工作空间
PX4_PATH="/workspace/Libraries/PX4-Autopilot" # PX4 固件根目录
MODEL_DIR="/workspace/catkin_ws/src/tutorials/tutorial_gazebo/models"
PKG_MODEL_DIR="${WS_PATH}/src/vision_ring/models"

# 2. Tmux 会话名称
SESSION_NAME="drone_test"

# 3. 仿真环境配置 (Window 0)
SIM_PKG="tutorial_gazebo"
SIM_LAUNCH="sim.launch"
WORLD_FILE="${WS_PATH}/src/vision_ring/worlds/task.world"

# 4. 算法节点配置 (Window 1)
ALGO_PKG="vision_ring"
ALGO_LAUNCH="vision_ring.launch"

# 5. 视觉检测节点配置 (Window 2)
ENABLE_VISION=true  # 设置为 true 自动启动 Python 脚本，设置为 false 则只留窗口不执行
VISION_SCRIPT_DIR="${WS_PATH}/src/vision_ring/yolo_detector/scripts"
VISION_SCRIPT_NAME="ring_detector.py"

# 6. 监听话题
TOPIC_TO_MONITOR="/mavros/local_position/pose"

# =========================================================================
#  [核心逻辑区] 
# =========================================================================

echo "🚀 正在启动 ROS Tmux 会话: [${SESSION_NAME}]"

# 1. 清理旧会话
tmux kill-session -t ${SESSION_NAME} 2>/dev/null
sleep 1 

# 2. 创建新会话
tmux new-session -d -s ${SESSION_NAME} -n "simulation"

# ----------------------------------------
# 窗口 0：仿真环境 (roscore + Gazebo)
# ----------------------------------------
tmux send-keys -t ${SESSION_NAME}:0 'roscore' C-m
tmux split-window -h -t ${SESSION_NAME}:0
tmux send-keys -t ${SESSION_NAME}:0.1 "sleep 3; \
source ${LIB_PATH}/devel/setup.bash; \
source ${WS_PATH}/devel/setup.bash; \
source ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${MODEL_DIR}:${PKG_MODEL_DIR}; \
export GAZEBO_MODEL_DATABASE_URI=\"\"; \
roslaunch ${SIM_PKG} ${SIM_LAUNCH} fcu_url:=\"udp://:14540@127.0.0.1:14557\" world:=\"${WORLD_FILE}\"" C-m
tmux select-layout -t ${SESSION_NAME}:0 tiled

# ----------------------------------------
# 窗口 1：算法与监控 (C++ 控制节点)
# ----------------------------------------
tmux new-window -t ${SESSION_NAME}:1 -n "algorithm"
tmux send-keys -t ${SESSION_NAME}:1 "sleep 6; source /opt/ros/noetic/setup.bash; rostopic echo ${TOPIC_TO_MONITOR}" C-m
tmux split-window -v -t ${SESSION_NAME}:1
tmux send-keys -t ${SESSION_NAME}:1.1 "sleep 7; source ${WS_PATH}/devel/setup.bash; roslaunch ${ALGO_PKG} ${ALGO_LAUNCH}" C-m
tmux select-layout -t ${SESSION_NAME}:1 tiled

# ----------------------------------------
# 窗口 2：视觉模块 (YOLO Python 节点)
# ----------------------------------------
tmux new-window -t ${SESSION_NAME}:2 -n "vision"
if [ "$ENABLE_VISION" = true ] ; then
    tmux send-keys -t ${SESSION_NAME}:2 "sleep 8; source /opt/ros/noetic/setup.bash; source ${WS_PATH}/devel/setup.bash; cd ${VISION_SCRIPT_DIR}; python3 ${VISION_SCRIPT_NAME}" C-m
else
    tmux send-keys -t ${SESSION_NAME}:2 "source /opt/ros/noetic/setup.bash; source ${WS_PATH}/devel/setup.bash; cd ${VISION_SCRIPT_DIR}; echo '✅ 视觉节点已禁用，随时可在此处手动运行'" C-m
fi

# ----------------------------------------
# 窗口 3：万能控制台 (打命令专用)
# ----------------------------------------
tmux new-window -t ${SESSION_NAME}:3 -n "terminal"
tmux send-keys -t ${SESSION_NAME}:3 "source /opt/ros/noetic/setup.bash; source ${WS_PATH}/devel/setup.bash; cd ${WS_PATH}; clear; echo '💻 环境已 source，可以开始起飞或者录制 rosbag 了！'" C-m

# ----------------------------------------
# 收尾
# ----------------------------------------
tmux select-window -t ${SESSION_NAME}:0
tmux attach-session -t ${SESSION_NAME}
