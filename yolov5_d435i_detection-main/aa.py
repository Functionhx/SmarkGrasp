import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
import time
import random
import yaml
import torch
from ultralytics import YOLO
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from utils.torch_utils import select_device, load_classifier, time_sync
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from utils.datasets import LoadStreams, LoadImages, letterbox
from models.experimental import attempt_load
import torch.backends.cudnn as cudnn

# 初始化ROS节点和发布器
rospy.init_node('realsense_yolo_detector', anonymous=True)
rgb_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
ir_left_pub = rospy.Publisher('/camera/infra1/image_raw', Image, queue_size=1)
ir_right_pub = rospy.Publisher('/camera/infra2/image_raw', Image, queue_size=1)
mask_pub = rospy.Publisher('/bottle_mask', Image, queue_size=1)
livox_infer_pub = rospy.Publisher("/infer_goal", PoseStamped, queue_size=10)
bridge = CvBridge()

# 定义坐标变换矩阵
livox_R_A2B = np.array([[ 0,  0,  1, 0.0],
                        [-1,  0,  0, -0.24],
                        [ 0, -1,  0, 0.20],
                        [ 0,  0,  0, 1]], dtype=np.float64)

# 加载YOLOv8分割模型
yolo_model = YOLO("/home/yh/tensorRT_Pro-YOLOv8/python/yolov8s-seg.pt")

# 配置RealSense管道
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 360, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 360, rs.format.y8, 30)
pipeline.start(config)

# 对齐设置
align_to = rs.stream.color
align = rs.align(align_to)

def get_aligned_images():
    """获取对齐的深度图和彩色图"""
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    # 获取相机内参
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

try:
    while not rospy.is_shutdown():
        # 获取对齐的图像与相机内参
        intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()
        if depth_image.size == 0 or color_image.size == 0:
            continue
        
        # 获取红外图像
        frames = pipeline.wait_for_frames()
        ir_left = frames.get_infrared_frame(1)
        ir_right = frames.get_infrared_frame(2)
        if not ir_left or not ir_right:
            continue
        
        # 转为numpy数组
        left_image = np.asanyarray(ir_left.get_data())
        right_image = np.asanyarray(ir_right.get_data())
        
        # YOLOv8分割推理
        t_start = time.time()
        result = yolo_model(color_image)[0]
        h, w = color_image.shape[:2]
        segmentation_mask = np.zeros((h, w), dtype=np.uint8)
        names = result.names
        bottle_class_id = None
        
        # 找到瓶子的类别ID
        for idx, name in names.items():
            if name == "bottle":
                bottle_class_id = idx
                break
        
        # 处理检测结果
        camera_xyz_list = []
        bottle_detected = False
        
        if bottle_class_id is not None and result.masks is not None:
            boxes = result.boxes.data.tolist()
            masks = result.masks
            for i, mask in enumerate(masks.data):
                label = int(boxes[i][5])
                if label == bottle_class_id:
                    mask_resized = mask.cpu().numpy().astype(np.uint8)
                    mask_resized = cv2.resize(mask_resized, (w, h))
                    segmentation_mask[mask_resized == 1] = 255
                    
                    # 计算掩码的中心点
                    contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        # 找到最大的轮廓
                        max_contour = max(contours, key=cv2.contourArea)
                        M = cv2.moments(max_contour)
                        if M["m00"] != 0:
                            ux = int(M["m10"] / M["m00"])
                            uy = int(M["m01"] / M["m00"])
                            
                            # 获取深度值并计算3D位置
                            dis = aligned_depth_frame.get_distance(ux, uy)
                            if dis > 0:  # 确保有有效的深度值
                                camera_xyz = rs.rs2_deproject_pixel_to_point(
                                    depth_intrin, (ux, uy), dis)
                                camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                                camera_xyz_list.append(camera_xyz)
                                bottle_detected = True
                                
                                # 在图像上标注中心点和坐标
                                cv2.circle(color_image, (ux, uy), 4, (255, 255, 255), 5)
                                cv2.putText(color_image, str(camera_xyz), (ux+20, uy+10), 0, 1,
                                            [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)
        
        t_end = time.time()
        
        # 发布雷达导航信息 - 只发布瓶子的位置（转换后）
        if bottle_detected:
            # 使用第一个检测到的瓶子
            camera_xyz = camera_xyz_list[0]
            
            # 创建齐次坐标向量 [x, y, z, 1]
            T = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0], dtype=np.float64)
            
            # 应用坐标变换
            livox_T_B = livox_R_A2B @ T
            
            # 提取变换后的位置（前三个元素）
            livox_pos_B = livox_T_B[:3]
            
            # 创建ROS消息并发布
            livox_infer_data = PoseStamped()
            livox_infer_data.header.stamp = rospy.Time.now()
            livox_infer_data.pose.position.x = livox_pos_B[0]
            livox_infer_data.pose.position.y = livox_pos_B[1]
            livox_infer_data.pose.position.z = livox_pos_B[2]
            livox_infer_pub.publish(livox_infer_data)
            
            # 打印转换前后的坐标
            print("\n===== 检测到瓶子 =====")
            print(f"原始相机坐标系: x={camera_xyz[0]:.3f}m, y={camera_xyz[1]:.3f}m, z={camera_xyz[2]:.3f}m")
            print(f"雷达坐标系: x={livox_pos_B[0]:.3f}m, y={livox_pos_B[1]:.3f}m, z={livox_pos_B[2]:.3f}m")
        else:
            print("未检测到瓶子")
        
        # 发布ROS话题
        rgb_pub.publish(bridge.cv2_to_imgmsg(color_image, "bgr8"))
        ir_left_pub.publish(bridge.cv2_to_imgmsg(left_image, "mono8"))
        ir_right_pub.publish(bridge.cv2_to_imgmsg(right_image, "mono8"))
        mask_pub.publish(bridge.cv2_to_imgmsg(segmentation_mask, "mono8"))
        
        # 添加fps显示
        fps = int(1.0 / (t_end - t_start))
        cv2.putText(color_image, text=f"FPS: {fps}", org=(50, 50),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=2,
                    lineType=cv2.LINE_AA, color=(0, 0, 255))
        
        # 显示图像
        cv2.imshow("RGB Detection", color_image)
        cv2.imshow("Bottle Mask", segmentation_mask)
        
        key = cv2.waitKey(1)
        if key == 27:  # ESC退出
            break

except Exception as e:
    print(f"发生错误: {e}")
    rospy.logerr(f"发生错误: {e}")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()