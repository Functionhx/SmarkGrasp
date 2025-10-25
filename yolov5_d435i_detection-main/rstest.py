'''
by yzh 2022.2.13
'''
# 导入依赖
import random
from utils.torch_utils import select_device, load_classifier, time_sync
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from utils.datasets import LoadStreams, LoadImages, letterbox
from models.experimental import attempt_load
import torch.backends.cudnn as cudnn
import torch

import pyrealsense2 as rs
import math
import yaml
import argparse
import os
import time
import numpy as np
import sys
from geometry_msgs.msg import PoseStamped
import cv2
# PyTorch
# YoloV5-PyTorch
import cv2
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


pipeline = rs.pipeline()  # 定义流程pipeline
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)  # 流程开始
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)


def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }'''

    # 保存内参到本地
    # with open('./intrinsics.json', 'w') as fp:
    #json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame


class YoloV5:
    def __init__(self, yolov5_yaml_path='config/yolov5s.yaml'):
        '''初始化'''
        # 载入配置文件
        with open(yolov5_yaml_path, 'r', encoding='utf-8') as f:
            self.yolov5 = yaml.load(f.read(), Loader=yaml.SafeLoader)
        # 随机生成每个类别的颜色
        self.colors = [[np.random.randint(0, 255) for _ in range(
            3)] for class_id in range(self.yolov5['class_num'])]
        # 模型初始化
        self.init_model()

    @torch.no_grad()
    def init_model(self):
        '''模型初始化'''
        # 设置日志输出
        set_logging()
        # 选择计算设备
        device = select_device(self.yolov5['device'])
        # 如果是GPU则使用半精度浮点数 F16
        is_half = device.type != 'cpu'
        # 载入模型
        model = attempt_load(
            self.yolov5['weight'], map_location=device)  # 载入全精度浮点数的模型
        input_size = check_img_size(
            self.yolov5['input_size'], s=model.stride.max())  # 检查模型的尺寸
        if is_half:
            model.half()  # 将模型转换为半精度
        # 设置BenchMark，加速固定图像的尺寸的推理
        cudnn.benchmark = True  # set True to speed up constant image size inference
        # 图像缓冲区初始化
        img_torch = torch.zeros(
            (1, 3, self.yolov5['input_size'], self.yolov5['input_size']), device=device)  # init img
        # 创建模型
        # run once
        _ = model(img_torch.half()
                  if is_half else img) if device.type != 'cpu' else None
        self.is_half = is_half  # 是否开启半精度
        self.device = device  # 计算设备
        self.model = model  # Yolov5模型
        self.img_torch = img_torch  # 图像缓冲区

    def preprocessing(self, img):
        '''图像预处理'''
        # 图像缩放
        # 注: auto一定要设置为False -> 图像的宽高不同
        img_resize = letterbox(img, new_shape=(
            self.yolov5['input_size'], self.yolov5['input_size']), auto=False)[0]
        # print("img resize shape: {}".format(img_resize.shape))
        # 增加一个维度
        img_arr = np.stack([img_resize], 0)
        # 图像转换 (Convert) BGR格式转换为RGB
        # 转换为 bs x 3 x 416 x
        # 0(图像i), 1(row行), 2(列), 3(RGB三通道)
        # ---> 0, 3, 1, 2
        # BGR to RGB, to bsx3x416x416
        img_arr = img_arr[:, :, :, ::-1].transpose(0, 3, 1, 2)
        # 数值归一化
        # img_arr =  img_arr.astype(np.float32) / 255.0
        # 将数组在内存的存放地址变成连续的(一维)， 行优先
        # 将一个内存不连续存储的数组转换为内存连续存储的数组，使得运行速度更快
        # https://zhuanlan.zhihu.com/p/59767914
        img_arr = np.ascontiguousarray(img_arr)
        return img_arr

    @torch.no_grad()
    def detect(self, img, canvas=None, view_img=True):
        '''模型预测'''
        # 图像预处理
        img_resize = self.preprocessing(img)  # 图像缩放
        self.img_torch = torch.from_numpy(img_resize).to(self.device)  # 图像格式转换
        self.img_torch = self.img_torch.half(
        ) if self.is_half else self.img_torch.float()  # 格式转换 uint8-> 浮点数
        self.img_torch /= 255.0  # 图像归一化
        if self.img_torch.ndimension() == 3:
            self.img_torch = self.img_torch.unsqueeze(0)
        # 模型推理
        t1 = time_sync()
        pred = self.model(self.img_torch, augment=False)[0]
        # pred = self.model_trt(self.img_torch, augment=False)[0]
        # NMS 非极大值抑制
        pred = non_max_suppression(pred, self.yolov5['threshold']['confidence'],
                                   self.yolov5['threshold']['iou'], classes=None, agnostic=False)
        t2 = time_sync()
        # print("推理时间: inference period = {}".format(t2 - t1))
        # 获取检测结果
        det = pred[0]
        gain_whwh = torch.tensor(img.shape)[[1, 0, 1, 0]]  # [w, h, w, h]

        if view_img and canvas is None:
            canvas = np.copy(img)
        xyxy_list = []
        conf_list = []
        class_id_list = []
        if det is not None and len(det):
            # 画面中存在目标对象
            # 将坐标信息恢复到原始图像的尺寸
            det[:, :4] = scale_coords(
                img_resize.shape[2:], det[:, :4], img.shape).round()
            for *xyxy, conf, class_id in reversed(det):
                class_id = int(class_id)
                xyxy_list.append(xyxy)
                conf_list.append(conf)
                class_id_list.append(class_id)
                if view_img:
                    # 绘制矩形框与标签
                    label = '%s %.2f' % (
                        self.yolov5['class_name'][class_id], conf)
                    self.plot_one_box(
                        xyxy, canvas, label=label, color=self.colors[class_id], line_thickness=3)
        return canvas, class_id_list, xyxy_list, conf_list

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        ''''绘制矩形框+标签'''
        tl = line_thickness or round(
            0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(
                label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3,
                        [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('realsense_yolo_detector', anonymous=True)
    
    # 创建ROS发布者
    rgb_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
    ir_left_pub = rospy.Publisher('/camera/infra1/image_raw', Image, queue_size=1)
    ir_right_pub = rospy.Publisher('/camera/infra2/image_raw', Image, queue_size=1)
    mask_pub = rospy.Publisher('/bottle_mask', Image, queue_size=1)
    livox_infer_pub = rospy.Publisher('/infer_goal', PoseStamped, queue_size=1)
    bridge = CvBridge()

    # 定义坐标变换矩阵
    livox_R_A2B = np.array([[ 0,  0,  1, 0.05],
                            [-1,  0,  0, -0.13],
                            [ 0, -1,  0, 0.13],
                            [ 0,  0,  0, 1]], dtype=np.float64)

    # 加载YOLO模型
    print("[INFO] 开始模型加载")
    yolo_seg_model = YOLO("yolov8s-seg.pt")  # 分割模型
    yolo_det_model = YoloV5(yolov5_yaml_path='config/yolov5s.yaml')  # 检测模型
    print("[INFO] 完成模型加载")

    # 配置RealSense管道 - 统一使用640x360分辨率
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 使用640x360分辨率
    config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 1, 640, 360, rs.format.y8, 30)
    config.enable_stream(rs.stream.infrared, 2, 640, 360, rs.format.y8, 30)
    config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
    
    # 创建对齐对象（深度对齐到彩色）
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    # 启动管道
    profile = pipeline.start(config)
    PUBLISH_INTERVAL = 0.1  # 发布间隔（秒）
    last_publish_time = time.time()

    try:
        # 获取深度传感器和内参
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        
        # 获取深度内参
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrin = depth_profile.get_intrinsics()
        
        # 设置分辨率变量
        W, H = 640, 360

        while not rospy.is_shutdown():
            current_time = time.time()
            # 获取并对齐帧
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            # 获取各数据流
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            ir_left = aligned_frames.get_infrared_frame(1)
            ir_right = aligned_frames.get_infrared_frame(2)
            
            if not all([color_frame, depth_frame, ir_left, ir_right]):
                continue

            # 转换为OpenCV格式
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            left_image = np.asanyarray(ir_left.get_data())
            right_image = np.asanyarray(ir_right.get_data())
            
            # ========== YOLOv8分割处理 ==========
            seg_start = time.time()
            seg_result = yolo_seg_model(color_image)[0]
            segmentation_mask = np.zeros((H, W), dtype=np.uint8)
            
            # 查找瓶子类ID
            bottle_class_id = None
            for idx, name in seg_result.names.items():
                if name == "bottle":
                    bottle_class_id = idx
                    break
            
            # 处理分割结果
            if bottle_class_id is not None and seg_result.masks is not None:
                boxes = seg_result.boxes.data.tolist()
                masks = seg_result.masks
                for i, mask in enumerate(masks.data):
                    label = int(boxes[i][5])
                    if label == bottle_class_id:
                        mask_resized = mask.cpu().numpy().astype(np.uint8)
                        mask_resized = cv2.resize(mask_resized, (W, H))
                        segmentation_mask[mask_resized == 1] = 255
            
            seg_end = time.time()
            
            # ========== YOLOv5检测处理 ==========
            det_start = time.time()
            canvas, class_id_list, xyxy_list, conf_list = yolo_det_model.detect(color_image)
            det_end = time.time()
            
            # 处理检测结果
            camera_xyz_list = []
            bottle_detected = False
            
            if xyxy_list:
                for i in range(len(xyxy_list)):
                    ux = int((xyxy_list[i][0] + xyxy_list[i][2]) / 2)
                    uy = int((xyxy_list[i][1] + xyxy_list[i][3]) / 2)
                    
                    # 确保坐标在图像范围内
                    ux = max(0, min(ux, W-1))
                    uy = max(0, min(uy, H-1))
                    
                    # 获取深度值并转换为米
                    dis = depth_frame.get_distance(ux, uy)
                    
                    # 反投影到3D空间
                    camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, [ux, uy], dis)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                    
                    class_id = class_id_list[i]
                    class_name = yolo_det_model.yolov5['class_name'][class_id]
                    
                    # 只处理瓶子
                    if class_name == "bottle":
                        cv2.circle(canvas, (ux, uy), 4, (255, 255, 255), 5)
                        cv2.putText(canvas, str(camera_xyz), (ux+20, uy+10), 0, 0.5,
                                    [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)
                        camera_xyz_list.append(camera_xyz)
                        bottle_detected = True
            
            # 发布瓶子位置（雷达坐标系）
            if current_time - last_publish_time >= PUBLISH_INTERVAL:
                if bottle_detected:
                    camera_xyz = camera_xyz_list[0]
                    T = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0], dtype=np.float64)
                    livox_T_B = livox_R_A2B @ T
                    livox_pos_B = livox_T_B[:3]
                    
                    livox_infer_data = PoseStamped()
                    livox_infer_data.header.stamp = rospy.Time.now()
                    livox_infer_data.header.frame_id = "livox_frame"
                    livox_infer_data.pose.position.x = livox_pos_B[0]
                    livox_infer_data.pose.position.y = livox_pos_B[1]
                    livox_infer_data.pose.position.z = 0
                    livox_infer_pub.publish(livox_infer_data)
                    
                    # 打印坐标信息
                    print(f"瓶子位置 - 相机坐标系: ({camera_xyz[0]:.3f}, {camera_xyz[1]:.3f}, {camera_xyz[2]:.3f})")
                    print(f"瓶子位置 - 雷达坐标系: ({livox_pos_B[0]:.3f}, {livox_pos_B[1]:.3f}, {livox_pos_B[2]:.3f})")

                    last_publish_time = current_time
                
                # ========== 发布ROS消息 ==========
                rgb_pub.publish(bridge.cv2_to_imgmsg(color_image, "bgr8"))
                ir_left_pub.publish(bridge.cv2_to_imgmsg(left_image, "mono8"))
                ir_right_pub.publish(bridge.cv2_to_imgmsg(right_image, "mono8"))
                mask_pub.publish(bridge.cv2_to_imgmsg(segmentation_mask, "mono8"))
                
            # ========== 显示结果 ==========
            # 计算并显示FPS
            seg_fps = 1.0 / (seg_end - seg_start + 1e-6)
            det_fps = 1.0 / (det_end - det_start + 1e-6)
            
            cv2.putText(canvas, f"Seg FPS: {seg_fps:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(canvas, f"Det FPS: {det_fps:.1f}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示图像
            cv2.imshow("RGB", color_image)
            cv2.imshow("Bottle Mask", segmentation_mask)
            cv2.imshow("Detection", canvas)
            
            key = cv2.waitKey(1)
            if key == 27:  # ESC退出
                break

    except Exception as e:
        import traceback
        print(f"发生错误: {e}")
        print(traceback.format_exc())
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()