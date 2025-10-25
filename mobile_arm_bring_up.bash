#!/bin/bash

# 初始化变量
MASKSENDER_PID=""
INFER_PID=""
ROSNODES_PIDS=()
ROS_STATUS_FILE="/tmp/ros_nodes.status"

# 清理函数 - 终止所有后台进程
cleanup() {
    echo "执行清理..."
    # 终止Python服务
    if [ -n "$MASKSENDER_PID" ]; then
        kill $MASKSENDER_PID 2>/dev/null
    fi
    if [ -n "$INFER_PID" ]; then
        kill $INFER_PID 2>/dev/null
    fi
    
    # 终止ROS节点
    pkill -f "roslaunch\|masksender\|infer2.py" 2>/dev/null
    
    # 清理状态文件
    rm -f $ROS_STATUS_FILE 2>/dev/null
    
    echo "清理完成"
    exit 1
}

# 设置错误处理：任何命令失败时调用cleanup
set -e
trap cleanup ERR

# === 第一部分：编译所有工作空间 ===
# cd /home/yh/point_lio
# catkin_make
# cd /home/yh/livox_ws
# catkin_make
# cd /home/yh/navi
# catkin_make
# cd /home/yh/ARX_X5-main/ROS/X5_ws
# catkin_make
# cd /home/yh

# === 第二部分：Conda环境处理 ===
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    CONDA_SCRIPT_PATH="$HOME/anaconda3/etc/profile.d/conda.sh"
    if [ -f "$CONDA_SCRIPT_PATH" ]; then
        source "$CONDA_SCRIPT_PATH"
    else
        echo "错误: 无法找到Conda初始化脚本！"
        echo "请手动设置Conda环境后执行"
        exit 1
    fi
fi

# === 第三部分：在graspness环境下运行Python程序 ===
echo "[阶段1] 启动GRASPNess感知服务"
{
echo "激活Conda环境: graspness"
conda activate graspness || {
    echo "错误: 无法激活graspness环境！"
    echo "可用环境: $(conda env list)"
    cleanup
}

# 检查Python脚本是否存在
if [ ! -f /home/yh/tensorRT_Pro-YOLOv8/python/masksender.py ]; then
    echo "错误: masksender.py 不存在!"
    cleanup
fi

if [ ! -f /home/yh/ASGrasp/infer2.py ]; then
    echo "错误: infer2.py 不存在!"
    cleanup
fi

# 启动服务并记录PID
cd /home/yh/yolov5_d435i_detection-main/
nohup python rstest.py > ~/rstest.log 2>&1 &
MASKSENDER_PID=$!
echo "启动 masksender.py (PID:$MASKSENDER_PID)"

cd /home/yh/ASGrasp/
nohup python infer2.py > ~/infer.log 2>&1 &
INFER_PID=$!
echo "启动 infer2.py (PID:$INFER_PID)"

# 验证进程是否运行
sleep 2
if ! ps -p $MASKSENDER_PID > /dev/null; then
    echo "错误: masksender.py 启动失败!"
    cleanup
fi

if ! ps -p $INFER_PID > /dev/null; then
    echo "错误: infer2.py 启动失败!"
    cleanup
fi

echo "Conda服务已启动成功"
}

# === 第四部分：执行系统级硬件初始化 ===
echo "[阶段2] 初始化硬件接口"
sudo /usr/local/bin/setup_can.sh || {
    echo "CAN初始化失败! 错误代码:$?"
    # cleanup
}

# === 第五部分：启动ROS感知导航节点 ===
echo "[阶段3] 启动感知与导航系统"
{
    cd /home/yh
    # 集中加载ROS环境
    source /opt/ros/noetic/setup.bash
    
    # 创建状态监控文件
    > $ROS_STATUS_FILE
    
    # 定义启动ROS节点的函数
    launch_ros_node() {
        local title=$1
        shift
        gnome-terminal --title "$title" -- bash -c "$*; exec bash"
        ROSNODES_PIDS+=($!)
        echo "启动 $title (终端PID:${ROSNODES_PIDS[-1]})"
    }
    
    # 启动节点
    source /home/yh/livox_ws/devel/setup.bash
    launch_ros_node "LIDAR驱动" "echo 'LIDAR驱动' >> $ROS_STATUS_FILE; roslaunch livox_ros_driver2 msg_MID360.launch"
    sleep 1

    source /home/yh/point_lio/devel/setup.bash
    launch_ros_node "SLAM建图" "echo 'SLAM建图' >> $ROS_STATUS_FILE; roslaunch point_lio mapping_mid360.launch"
    sleep 1
    
    source /home/yh/navi/devel/setup.bash
    launch_ros_node "点云聚类" "echo '点云聚类' >> $ROS_STATUS_FILE; roslaunch depth_cluster_ros depth_cluster_ros.launch"
    sleep 1
    launch_ros_node "路径规划" "echo '路径规划' >> $ROS_STATUS_FILE; roslaunch local_planner navigation.launch"
    sleep 1
    launch_ros_node "底盘驱动" "echo '底盘驱动' >> $ROS_STATUS_FILE; roslaunch yhs_can_control yhs_can_control.launch"
    launch_ros_node "XXX" "echo 'XXX' >> $ROS_STATUS_FILE; roslaunch nav_service nav_service.launch"
    
    
 
    # 验证节点启动
    echo "等待节点初始化(5秒)..."
    sleep 3
    
    if [ $(wc -l < $ROS_STATUS_FILE) -ne 6 ]; then
        echo "警告: 部分ROS节点未正常启动!"
        echo "当前状态: $(cat $ROS_STATUS_FILE)"
        cleanup
    fi
}

# === 第六部分：启动机械臂控制系统 ===
echo "[阶段4] 启动机械臂控制"
{
    source /home/yh/ARX_X5-main/ROS/X5_ws/devel/setup.bash
    
    launch_ros_node "机械臂控制" "roslaunch arx_x5_controller open_single_arm.launch"
    
    echo "等待机械臂就绪(10秒)..."
    sleep 3
    
    launch_ros_node "轨迹生成" "roslaunch arx_x5_controller trajectory_generator.launch"
    
    echo "所有系统已启动成功!"
    echo "请检查终端窗口："
    echo "  - LIDAR驱动, SLAM建图, 点云聚类, 路径规划，底盘驱动"
    echo "  - 机械臂控制, 轨迹生成"
}

# === 最终部分：保持脚本运行 ===
echo "系统运行中... 按 Ctrl+C 终止"
trap 'cleanup; exit' SIGINT
while true; do sleep 1; done
