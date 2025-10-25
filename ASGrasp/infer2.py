from infer_mvs_2layer_gsnet import MVSGSNetEval
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import threading
import gc
import time
import torch
import numpy as np
import os
import sys
import cv2
import open3d as o3d
import math
from graspnetAPI.graspnet_eval import GraspGroup
sys.path.append("./gsnet/pointnet2")

sys.path.append("./gsnet/utils")
sys.path.append("./src/core_multilayers")
sys.path.append("/home/yh/ARX_X5-main/ROS/X5_ws/devel/lib/python3/dist-packages")
from gsnet.models.graspnet import GraspNet, pred_decode
from raft_mvs_multilayers import RAFTMVS_2Layer
from dataset.graspnet_dataset import minkowski_collate_fn
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image, get_shape_aware_workspace, get_workspace_mask
from D415_camera import CameraMgr
from arm_control.msg import PosCmd
from geometry_msgs.msg import PoseStamped

# DEBUG_VIS = True
DEBUG_VIS = False
os.environ['OMP_NUM_THREADS'] = '20'
infer_pub = rospy.Publisher("/infer_cmd", PosCmd, queue_size=10)

class RealTimeGraspDetector:
    def __init__(self, eval_model):
        self.eval = eval_model
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.rgb = None
        self.ir1 = None
        self.ir2 = None
        self.mask = None
        self.last_infer_time = 0
        self.processing = False
        self.arrived = False
        
        # 存储上一次有效的抓取结果
        self.last_valid_gg = None
        self.last_valid_pose_B_7x1 = None
        self.last_valid_grasp_time = 0
        
        rospy.Subscriber("/infer_status", Bool, self.arrived_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/infra1/image_raw", Image, self.ir1_callback)
        rospy.Subscriber("/camera/infra2/image_raw", Image, self.ir2_callback)
        rospy.Subscriber("/bottle_mask", Image, self.mask_callback)

    def arrived_callback(self, msg):
        with self.lock:
            self.arrived = msg.data
            if self.arrived and self.last_valid_gg is not None:
                rospy.loginfo("小车已到达目标位置，准备发布抓取命令")
                self.publish_grasp_command(self.last_valid_gg, self.last_valid_pose_B_7x1)
                rospy.loginfo("已使用上一次有效的抓取结果发布命令")

    def rgb_callback(self, msg):
        with self.lock:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.try_infer()

    def ir1_callback(self, msg):
        with self.lock:
            self.ir1 = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.try_infer()

    def ir2_callback(self, msg):
        with self.lock:
            self.ir2 = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.try_infer()

    def mask_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
            if cv_img is None:
                rospy.logwarn("接收到空mask，使用默认全1mask")
                with self.lock:
                    self.mask = None
                return
                
            with self.lock:
                self.mask = cv_img
                self.try_infer()
        except Exception as e:
            rospy.logwarn(f"Mask回调异常: {e}")
            with self.lock:
                self.mask = None

    def try_infer(self):
        # 检查是否有足够的图像数据
        has_images = self.rgb is not None and self.ir1 is not None and self.ir2 is not None
        has_mask = self.mask is not None
        can_process = not self.processing and has_images
        
        if not can_process:
            return
        
        now = time.time()
        if now - self.last_infer_time < 0.5:  # 控制推理频率
            return
        
        rgb = self.rgb.copy()
        ir1 = self.ir1.copy()
        ir2 = self.ir2.copy()
        mask = self.mask.copy() if has_mask else None
        
        self.processing = True
        self.last_infer_time = now

        def infer_thread():
            try:
                rospy.loginfo("收到全部图像，开始推理...")
                result = self.eval.infer_cv(rgb, ir1, ir2, mask)
                
                if result is not None:
                    gg, pose_B_7x1 = result
                    # 更新有效抓取结果
                    with self.lock:
                        self.last_valid_gg = gg
                        self.last_valid_pose_B_7x1 = pose_B_7x1
                        self.last_valid_grasp_time = time.time()
            except Exception as e:
                rospy.logerr(f"推理异常: {e}")
            finally:
                gc.collect()
                torch.cuda.empty_cache()
                with self.lock:
                    self.processing = False

        threading.Thread(target=infer_thread, daemon=True).start()
    
    def publish_grasp_command(self, gg, pose_B_7x1):
        if gg is None or len(gg) == 0:
            rospy.logwarn("没有有效的抓取结果可发布")
            return
            
        grasp = gg[0]
        pos_B = pose_B_7x1[:3]
        euler = pose_B_7x1[3:6]
        width = pose_B_7x1[6]

        infer_data = PosCmd()
        infer_data.x = pos_B[0]
        infer_data.y = pos_B[1]
        infer_data.z = pos_B[2]                          
        infer_data.roll = euler[0]
        infer_data.pitch = euler[1]
        infer_data.yaw = euler[2]
        infer_data.gripper = width
        infer_pub.publish(infer_data)
        rospy.loginfo("已发布抓取命令")

def rotation_matrix_to_euler_angles(R):
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.degrees([x, y, z])

def load_image_cv(img_cv):
    if img_cv is None:
        rospy.logwarn("尝试加载空图像")
        return None
        
    if not isinstance(img_cv, np.ndarray):
        rospy.logwarn(f"输入不是numpy数组: {type(img_cv)}")
        return None
        
    if img_cv.dtype != np.uint8:
        rospy.logwarn(f"输入dtype是 {img_cv.dtype}, 期望uint8")
        return None
        
    if len(img_cv.shape) == 2:
        img = np.stack([img_cv]*3, axis=-1)
    else:
        img = img_cv
        
    img = img[:, :, ::-1].copy()
    img = torch.from_numpy(img).permute(2, 0, 1).float()
    return img[None].cuda()

def infer_cv(self, rgb_img, ir1_img, ir2_img, mask_img, zrot=None):
    start_time = time.time()
    with torch.no_grad():
        # 加载图像并检查有效性
        color = load_image_cv(rgb_img)
        ir1 = load_image_cv(ir1_img)
        ir2 = load_image_cv(ir2_img)
        
        if color is None or ir1 is None or ir2 is None:
            rospy.logwarn("图像加载失败，跳过推理")
            return None, None

        # 深度估计
        depth_up = self.mvs_net(color, ir1, ir2, self.proj_matrices.clone(), self.dmin, self.dmax, iters=self.args.valid_iters, test_mode=True)
        depth_up = (depth_up).squeeze()
        depth_2layer = depth_up.detach().cpu().numpy().squeeze()

        # 检查深度图有效性
        if depth_2layer.size == 0 or depth_2layer.shape[0] < 2:
            rospy.logwarn("深度估计失败，跳过推理")
            return None, None
            
        depth = depth_2layer[0]
        
        # 创建点云
        cloud = create_point_cloud_from_depth_image(depth, self.camera, organized=True)
        if cloud is None or cloud.size == 0:
            rospy.logwarn("点云创建失败，目标可能离开视野")
            return None, None
            
        depth_mask = (depth>0.20) & (depth<5)

        # 处理mask缺失的情况
        if mask_img is None:
            rospy.logwarn("未收到mask，使用默认全1mask")
            seg = np.ones(depth.shape)
        else:
            seg = mask_img

        workspace_mask = get_shape_aware_workspace(cloud, seg, organized=True, expansion=0.002)
        mask = (depth_mask & workspace_mask)

        cloud_masked = cloud[mask]
        
        # 检查点云数据是否足够
        if cloud_masked.size == 0:
            rospy.logwarn("工作区内无点云数据，目标可能离开视野")
            return None, None
            
        color_np = (color).squeeze().detach().cpu().numpy().squeeze()
        color_np = np.transpose(color_np, [1,2,0])
        color_masked = color_np[mask]
        
        # 采样点云
        num_points = min(2500, len(cloud_masked))
        if num_points < 100:
            rospy.logwarn(f"点云数据不足({num_points}点)，跳过推理")
            return None, None
            
        idxs = np.random.choice(len(cloud_masked), num_points, replace=False)
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]

        # 添加第二层点云
        depth1 = depth_2layer[1]
        cloud1 = create_point_cloud_from_depth_image(depth1, self.camera, organized=True)
        if cloud1 is None or cloud1.size == 0:
            rospy.logwarn("第二层点云创建失败")
            objectness_label = np.ones([num_points, ])
        else:
            depth_mask1 = (depth1 > 0.10) & (depth1 < 5) & (depth1-depth>0.001)
            mask1 = (depth_mask1 & workspace_mask)
            cloud_masked1 = cloud1[mask1]
            comp_num_pt = 2500
            
            if len(cloud_masked1) > 0:
                rospy.loginfo(f'completed_point_cloud : {len(cloud_masked1)}')
                if len(cloud_masked1) >= comp_num_pt:
                    idxs = np.random.choice(len(cloud_masked1), comp_num_pt, replace=False)
                else:
                    idxs1 = np.arange(len(cloud_masked1))
                    idxs2 = np.random.choice(len(cloud_masked1), comp_num_pt - len(cloud_masked1), replace=True)
                    idxs = np.concatenate([idxs1, idxs2], axis=0)
                completed_sampled = cloud_masked1[idxs]

                cloud_sampled = np.concatenate([cloud_sampled, completed_sampled], axis=0)
                objectness_label = np.concatenate([np.ones([num_points, ]), (-1)*np.ones([comp_num_pt, ])], axis=0)
                cloud_masked = np.concatenate([cloud_masked, cloud_masked1], axis=0)
            else:
                objectness_label = np.ones([num_points, ])

        # 检查最终点云数据是否足够
        if cloud_sampled.size == 0 or len(cloud_sampled) < 100:
            rospy.logwarn("最终点云数据不足，跳过推理")
            return None, None
            
        ret_dict = {'point_clouds': cloud_sampled.astype(np.float32),
                    'coors': cloud_sampled.astype(np.float32) / 0.005,
                    'feats': np.ones_like(cloud_sampled).astype(np.float32),
                    'full_point_clouds': cloud_masked.astype(np.float32),
                    'objectness_label': objectness_label.astype(np.int32),
                    }

        batch_data = minkowski_collate_fn([ret_dict])
        for key in batch_data:
            if 'list' in key:
                for i in range(len(batch_data[key])):
                    for j in range(len(batch_data[key][i])):
                        batch_data[key][i][j] = batch_data[key][i][j].cuda()
            else:
                batch_data[key] = batch_data[key].cuda()
        try:
            end_points = self.gsnet(batch_data)
        except Exception as e:
            rospy.logerr(f"GraspNet推理异常: {e}")
            return None, None
        grasp_preds = pred_decode(end_points)

        torch.cuda.empty_cache()
        preds = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(preds)

        # 碰撞检测
        if self.args.collision_thresh > 0:
            cloud = ret_dict['full_point_clouds']
            cloud = cloud_masked.astype(np.float32)
            
            # 检查碰撞检测点云是否有效
            if cloud.size == 0:
                rospy.logwarn("碰撞检测点云为空，跳过碰撞检测")
            else:
                mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=self.args.voxel_size_cd)
                collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.args.collision_thresh)
                gg = gg[~collision_mask]

        gg = gg.nms()
        gg = gg.sort_by_score()
        count = gg.__len__()
        if count < 1:
            rospy.logwarn("未检测到有效抓取")
            return None, None
            
        gg = gg[:1]  # 只取最佳抓取
        
        pose_B_7x1 = None
        if gg and len(gg) > 0:
            grasp = gg[0]
            position = grasp.translation
            rotation_matrix = grasp.rotation_matrix
            T = np.eye(4)
            T[:3, :3] = grasp.rotation_matrix
            T[:3, 3] = grasp.translation
            
            R_A2B = np.array([[ 0,  0,  1, 0.052],
                              [-1,  0,  0, 0.032],
                              [ 0, -1,  0, 0.055],
                              [ 0,  0,  0, 1]], dtype=np.float64)
            T_B = R_A2B @ T
            pos_B = T_B[:3, 3]
            rot_B = T_B[:3, :3]
            
            sy = np.sqrt(rot_B[0,0]**2 + rot_B[1,0]**2)
            singular = sy < 1e-6
            if not singular:
                roll = np.arctan2(rot_B[2,1], rot_B[2,2])
                pitch = np.arctan2(-rot_B[2,0], sy)
                yaw = np.arctan2(rot_B[1,0], rot_B[0,0])
            else:
                roll = np.arctan2(-rot_B[1,2], rot_B[1,1])
                pitch = np.arctan2(-rot_B[2,0], sy)
                yaw = 0.0
                
            pose_B_7x1 = np.hstack([pos_B, [roll, pitch, yaw], grasp.width])
            
            rospy.loginfo(f"检测到有效抓取，置信度: {grasp.score:.4f}")
        else:
            rospy.logwarn("未检测到有效抓取姿态")
            return None, None

        if DEBUG_VIS:
            grippers = gg.to_open3d_geometry_list()
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(ret_dict['point_clouds'].astype(np.float32))
            point_cloud.colors = o3d.utility.Vector3dVector(color_sampled.astype(np.float32)/255.0)
            o3d.visualization.draw_geometries([point_cloud, *grippers])

        end_time = time.time()
        rospy.loginfo(f"本次推理耗时: {end_time - start_time:.3f} 秒")
        return gg, pose_B_7x1

# 绑定新方法到类
MVSGSNetEval.infer_cv = infer_cv

if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
    parser.add_argument('--valid_iters', type=int, default=16, help='number of flow-field updates during forward pass')
    parser.add_argument('--hidden_dims', nargs='+', type=int, default=[128] * 3,
                        help="hidden state and context dimensions")
    parser.add_argument('--corr_implementation', choices=["reg", "alt", "reg_cuda", "alt_cuda"], default="reg",
                        help="correlation volume implementation")
    parser.add_argument('--shared_backbone', action='store_true',
                        help="use a single backbone for the context and feature encoders")
    parser.add_argument('--corr_levels', type=int, default=4, help="number of levels in the correlation pyramid")
    parser.add_argument('--corr_radius', type=int, default=4, help="width of the correlation pyramid")
    parser.add_argument('--n_downsample', type=int, default=2, help="resolution of the disparity field (1/2^K)")
    parser.add_argument('--context_norm', type=str, default="batch", choices=['group', 'batch', 'instance', 'none'],
                        help="normalization of context encoder")
    parser.add_argument('--slow_fast_gru', action='store_true', help="iterate the low-res GRUs more frequently")
    parser.add_argument('--n_gru_layers', type=int, default=3, help="number of hidden GRU levels")
    parser.add_argument('--num_sample', type=int, default=96, help="number of depth levels")
    parser.add_argument('--depth_min', type=float, default=0.1, help="number of levels in the correlation pyramid")
    parser.add_argument('--depth_max', type=float, default=5.0, help="width of the correlation pyramid")
    parser.add_argument('--train_2layer', default=True, help="")
    parser.add_argument('--restore_ckpt', default=f'./checkpoints/raftmvs_2layer.pth', help="restore checkpoint")
    parser.add_argument('--checkpoint_path', default='./checkpoints/minkuresunet_epoch10.tar')
    parser.add_argument('--seed_feat_dim', default=512, type=int, help='Point wise feature dim')
    parser.add_argument('--num_point', type=int, default=15000, help='Point Number [default: 15000]')
    parser.add_argument('--batch_size', type=int, default=1, help='Batch Size during inference [default: 1]')
    parser.add_argument('--voxel_size', type=float, default=0.005, help='Voxel Size for sparse convolution')
    parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
    parser.add_argument('--voxel_size_cd', type=float, default=0.01, help='Voxel Size for collision detection')
    parser.add_argument('--graspness_threshold', type=float, default=0, help='graspness threshold')
    
    args = parser.parse_args()
    eval = MVSGSNetEval(args)
    rospy.init_node('graspnet_realtime_infer', anonymous=True)
    detector = RealTimeGraspDetector(eval)
    rospy.loginfo("等待ROS图像话题...")
    rospy.spin()