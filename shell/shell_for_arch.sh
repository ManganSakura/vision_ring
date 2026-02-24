#!/bin/bash

# =========================================================================
#  [配置区] 根据项目需求修改变量
# =========================================================================

# 1. 路径配置
WS_PATH="/workspace/project_ws"               # 应用层工作空间
LIB_PATH="/workspace/catkin_ws"               # 底层库工作空间
PX4_PATH="/workspace/Libraries/PX4-Autopilot" # PX4 固件根目录
# 【新增/优化】将模型路径全部提炼到这里
MODEL_DIR="/workspace/catkin_ws/src/tutorials/tutorial_gazebo/models"
# 你的包内模型路径，使用变量拼接，实现“牵一发而动全身”
PKG_MODEL_DIR="${WS_PATH}/src/vision_ring/models"

# 2. Tmux 会话名称
SESSION_NAME="drone_test"

# 3. 仿真环境配置 (Window 1)
SIM_PKG="tutorial_gazebo"
SIM_LAUNCH="sim.launch"
WORLD_FILE="${WS_PATH}/src/vision_ring/worlds/task.world"
# WORLD_FILE="worlds/empty.world" # 使用空世界进行测试，避免模型加载问题

# 4. 算法节点配置 (Window 2)
ALGO_PKG="vision_ring"
ALGO_LAUNCH="vision_ring.launch"

# 5. 监听话题
TOPIC_TO_MONITOR="/mavros/local_position/pose"


# =========================================================================
#  [核心逻辑区] (严禁在此处硬编码任何业务逻辑参数)
# =========================================================================

echo "正在启动 ROS Tmux 会话: [${SESSION_NAME}]"
echo "工作空间: ${WS_PATH}"

# 1. 清理旧会话
tmux kill-session -t ${SESSION_NAME} 2>/dev/null
sleep 1 

# 2. 创建新会话
tmux new-session -d -s ${SESSION_NAME} -n "simulation"

# ----------------------------------------
# 窗口 1：仿真环境 (roscore + Gazebo)
# ----------------------------------------
# Pane 0: roscore
tmux send-keys -t ${SESSION_NAME}:0 'roscore' C-m

# Pane 1: 启动仿真 (包含 PX4 环境配置修复)
tmux split-window -h -t ${SESSION_NAME}:0

# 【核心修复】：追加了 source ${WS_PATH}/devel/setup.bash
# 【接口调用】：在 roslaunch 尾部优雅地传入了 world:=\"${WORLD_FILE}\"
tmux send-keys -t ${SESSION_NAME}:0.1 "sleep 3; \
source ${LIB_PATH}/devel/setup.bash; \
source ${WS_PATH}/devel/setup.bash; \
source ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${MODEL_DIR}:${PKG_MODEL_DIR}; \
export GAZEBO_MODEL_DATABASE_URI=\"\"; \
roslaunch ${SIM_PKG} ${SIM_LAUNCH} fcu_url:=\"udp://:14540@127.0.0.1:14557\" world:=\"${WORLD_FILE}\"" C-m


# 布局整理
tmux select-layout -t ${SESSION_NAME}:0 tiled

# ----------------------------------------
# 窗口 2：算法与监控 (Algorithm + Monitor)
# ----------------------------------------
tmux new-window -t ${SESSION_NAME}:1 -n "algorithm"

# Pane 0: 话题监控
tmux send-keys -t ${SESSION_NAME}:1 "sleep 6; source /opt/ros/noetic/setup.bash; rostopic echo ${TOPIC_TO_MONITOR}" C-m

# Pane 1: 算法节点
tmux split-window -v -t ${SESSION_NAME}:1
tmux send-keys -t ${SESSION_NAME}:1.1 "sleep 7; source ${WS_PATH}/devel/setup.bash; roslaunch ${ALGO_PKG} ${ALGO_LAUNCH}" C-m

# 布局整理
tmux select-layout -t ${SESSION_NAME}:1 tiled

# ----------------------------------------
# 收尾
# ----------------------------------------
tmux select-window -t ${SESSION_NAME}:0
tmux attach-session -t ${SESSION_NAME}
