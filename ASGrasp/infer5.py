from infer_mvs_2layer_gsnet import MVSGSNetEval
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # 导入Bool消息类型
from cv_bridge import CvBridge
import threading
import gc
import time
import torch
import numpy as np
import os
import sys
import numpy as np
import torch
import cv2
import rospy
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

# DEBUG_VIS = True  # 实时建议关闭可视化
DEBUG_VIS = False  # 实时建议关闭可视化
os.environ['OMP_NUM_THREADS'] = '20'  # 限制线程数
infer_pub = rospy.Publisher("/infer_cmd", PosCmd, queue_size=10)
# livox_infer_pub = rospy.Publisher("/infer_goal", PoseStamped, queue_size=10)

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
        self.arrived = False  # 添加小车到达状态标志

        # 订阅小车到达话题
        rospy.Subscriber("/infer_status", Bool, self.arrived_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/infra1/image_raw", Image, self.ir1_callback)
        rospy.Subscriber("/camera/infra2/image_raw", Image, self.ir2_callback)
        rospy.Subscriber("/bottle_mask", Image, self.mask_callback)

    def arrived_callback(self, msg):
        with self.lock:
            self.arrived = msg.data
            if self.arrived:
                rospy.loginfo("小车已到达目标位置，准备发布抓取命令")

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
                    self.mask = None  # 将在infer_cv中处理
                return
                
            with self.lock:
                self.mask = cv_img
                self.try_infer()
        except Exception as e:
            rospy.logwarn(f"Mask回调异常: {e}")
            with self.lock:
                self.mask = None

    # def mask_callback(self, msg):
    #     with self.lock:
    #         self.mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
    #         self.try_infer()

    def try_infer(self):
        if (self.rgb is not None and self.ir1 is not None and self.ir2 is not None and self.mask is not None and not self.processing):
            now = time.time()
            if now - self.last_infer_time < 0.5:  # 控制推理频率
                return
            rgb = self.rgb.copy()
            ir1 = self.ir1.copy()
            ir2 = self.ir2.copy()
            mask = self.mask.copy()
            self.processing = True
            self.last_infer_time = now
        else:
            return

        def infer_thread():
                try:
                    rospy.loginfo("收到全部图像，开始推理...")
                    result = self.eval.infer_cv(rgb, ir1, ir2, mask, self.arrived)  # 传递到达状态
                    if result is not None:
                        self.last_valid_grasp_time = time.time()
                except Exception as e:
                    rospy.logerr(f"推理异常: {e}")
                finally:
                    gc.collect()
                    torch.cuda.empty_cache()
                    with self.lock:
                        self.processing = False
                        # 不清空图像缓存，继续使用最新图像
                        # self.rgb = self.ir1 = self.ir2 = self.mask = None

        threading.Thread(target=infer_thread, daemon=True).start()

def rotation_matrix_to_euler_angles(R):
    """
    将旋转矩阵R转换为欧拉角（roll, pitch, yaw），返回角度制
    """
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
    # 转为角度
    return np.degrees([x, y, z])

def load_image_cv(img_cv):
    assert img_cv is not None, "Input image is None"
    assert isinstance(img_cv, np.ndarray), "Input is not numpy array"
    assert img_cv.dtype == np.uint8, f"Input dtype is {img_cv.dtype}, expected uint8"
    # 可选：降低分辨率
    # img_cv = cv2.resize(img_cv, (320, 180))
    if len(img_cv.shape) == 2:
        img = np.stack([img_cv]*3, axis=-1)
    else:
        img = img_cv
    img = img[:, :, ::-1].copy()
    img = torch.from_numpy(img).permute(2, 0, 1).float()
    return img[None].cuda()

def infer_cv(self, rgb_img, ir1_img, ir2_img, mask_img, arrived, zrot=None):
    start_time = time.time()  # 记录开始时间
    with torch.no_grad():
        color = load_image_cv(rgb_img)
        ir1 = load_image_cv(ir1_img)
        ir2 = load_image_cv(ir2_img)

        depth_up = self.mvs_net(color, ir1, ir2, self.proj_matrices.clone(), self.dmin, self.dmax, iters=self.args.valid_iters, test_mode=True)
        depth_up = (depth_up).squeeze()
        depth_2layer = depth_up.detach().cpu().numpy().squeeze()

        depth = depth_2layer[0]
        cloud = create_point_cloud_from_depth_image(depth, self.camera, organized=True)
        depth_mask = (depth>0.20) & (depth<5)

        if mask_img is None:
            seg = np.ones(depth.shape)
        else:
            if len(mask_img.shape) == 3:
                seg = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
            else:
                seg = mask_img

        workspace_mask = get_shape_aware_workspace(cloud, seg, organized=True, expansion=0.002)
        mask = (depth_mask & workspace_mask)

        cloud_masked = cloud[mask]
        color_np = (color).squeeze().detach().cpu().numpy().squeeze()
        color_np = np.transpose(color_np, [1,2,0])
        color_masked = color_np[mask]
        idxs = np.random.choice(len(cloud_masked), min(2500, len(cloud_masked)), replace=False)
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]

        #add second layer
        depth1 = depth_2layer[1]
        cloud1 = create_point_cloud_from_depth_image(depth1, self.camera, organized=True)
        depth_mask1 = (depth1 > 0.10) & (depth1 < 5) & (depth1-depth>0.001)
        mask1 = (depth_mask1 & workspace_mask)
        cloud_masked1 = cloud1[mask1]
        comp_num_pt = 2500
        if (len(cloud_masked1) > 0):
            rospy.loginfo(f'completed_point_cloud : {len(cloud_masked1)}')
            if len(cloud_masked1) >= (comp_num_pt):
                idxs = np.random.choice(len(cloud_masked1), comp_num_pt, replace=False)
            else:
                idxs1 = np.arange(len(cloud_masked1))
                idxs2 = np.random.choice(len(cloud_masked1), comp_num_pt - len(cloud_masked1), replace=True)
                idxs = np.concatenate([idxs1, idxs2], axis=0)
            completed_sampled = cloud_masked1[idxs]

            cloud_sampled = np.concatenate([cloud_sampled, completed_sampled], axis=0)
            objectness_label = np.concatenate([np.ones([2500, ]), (-1)*np.ones([comp_num_pt, ])], axis=0)
            cloud_masked = np.concatenate([cloud_masked, cloud_masked1], axis=0)
        else:
            objectness_label = np.ones([2500, ])

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
            return None
        grasp_preds = pred_decode(end_points)

        torch.cuda.empty_cache()
        preds = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(preds)

        if self.args.collision_thresh > 0:
            cloud = ret_dict['full_point_clouds']
            cloud = cloud_masked.astype(np.float32)
            mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=self.args.voxel_size_cd)
            collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.args.collision_thresh)
            gg = gg[~collision_mask]

        gg = gg.nms()
        gg = gg.sort_by_score()
        count = gg.__len__()
        if count <1:
            rospy.logwarn("未检测到有效抓取")
            return None
        if count > 1:
            count = 1
        gg = gg[:count]
        pose_B_7x1 = None
        if gg is not None and len(gg) > 0:
            grasp = gg[0]
            position = grasp.translation
            rotation_matrix = grasp.rotation_matrix
            # roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)
            T = np.eye(4)                # 构造 4×4 单位阵
            T[:3, :3] = grasp.rotation_matrix  # 填旋转部分
            T[:3, 3]  = grasp.translation      # 填平移部分
            rospy.loginfo("\n===== 夹爪 4×4 矩阵 =====")
            np.set_printoptions(precision=4, suppress=True)
            rospy.loginfo(T)

            R_A2B = np.array([[ 0,  0,  1, 0.075],
                              [-1,  0,  0, 0.032],
                              [ 0, -1,  0, 0.040],
                              [ 0,  0,  0, 1]], dtype=np.float64)
            T_B = R_A2B @ T            # 4×4 齐次变换
            pos_B = T_B[:3, 3]
            rot_B = T_B[:3, :3]
            sy   = np.sqrt(rot_B[0,0]**2 + rot_B[1,0]**2)
            singular = sy < 1e-6
            if not singular:
                roll  = np.arctan2(rot_B[2,1], rot_B[2,2])
                pitch = np.arctan2(-rot_B[2,0], sy)
                yaw   = np.arctan2(rot_B[1,0], rot_B[0,0])
            else:                       # gimbal lock
                roll  = np.arctan2(-rot_B[1,2], rot_B[1,1])
                pitch = np.arctan2(-rot_B[2,0], sy)
                yaw   = 0.0
            pose_B_7x1 = np.hstack([pos_B, [roll, pitch, yaw],grasp.width])

            # livox_R_A2B = np.array([[ 0,  0,  1, 0.0],
            #                   [-1,  0,  0, -0.24],
            #                   [ 0, -1,  0, 0.20],
            #                   [ 0,  0,  0, 1]], dtype=np.float64)
            # livox_T_B = livox_R_A2B @ T
            # livox_pos_B = livox_T_B[:2, 3]  
            # rospy.loginfo("\n===== 雷达下的 2D 位置 =====")
            # rospy.loginfo(livox_pos_B)
            
            # # 发布小车导航信息
            # livox_infer_data = PoseStamped()
            # livox_infer_data.pose.position.x = livox_pos_B[0]
            # livox_infer_data.pose.position.y = livox_pos_B[1]
            # livox_infer_data.pose.position.z = 0.0
            # livox_infer_pub.publish(livox_infer_data)
            # print(f"已发布雷达导航信息: {livox_infer_data.pose.position.x}, {livox_infer_data.pose.position.y}")
            # rospy.loginfo("\n===== 坐标系 B 下的 6×1 位姿 =====")
            # np.set_printoptions(precision=4, suppress=True)
            # rospy.loginfo(pose_B_7x1)
            
            # 只有当小车到达时才发布抓取命令
            if arrived:
                infer_data = PosCmd()
                infer_data.x = pos_B[0]
                infer_data.y = pos_B[1]
                infer_data.z = pos_B[2]                          
                infer_data.roll = roll
                infer_data.pitch = pitch
                infer_data.yaw = yaw
                infer_data.gripper = grasp.width
                infer_pub.publish(infer_data)
                rospy.loginfo("已发布抓取命令（小车已到达）")
            else:
                rospy.loginfo("检测到有效抓取，但小车尚未到达，等待中...")
            
        #     rospy.loginfo("\n===== 抓取姿态信息(camera) =====")
        #     rospy.loginfo(f"夹爪位置 (x, y, z): ({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}) 米")
        #     rospy.loginfo(f"夹爪姿态 (弧度): roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")
        #     rospy.loginfo(f"夹爪张开宽度: {grasp.width:.4f} 米")
        #     rospy.loginfo(f"抓取置信度: {grasp.score:.4f}")
        # else:
        #     rospy.logwarn("未检测到有效抓取")   

        if DEBUG_VIS:
            grippers = gg.to_open3d_geometry_list()
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(ret_dict['point_clouds'].astype(np.float32))
            point_cloud.colors = o3d.utility.Vector3dVector(color_sampled.astype(np.float32)/255.0)
            o3d.visualization.draw_geometries([point_cloud, *grippers])

        gg = gg[:1]
        rospy.loginfo(f"grasp width : {gg.widths}")
        rospy.loginfo(f"grasp score : {gg.scores}")
        end_time = time.time()  # 记录结束时间
        rospy.loginfo(f"本次推理耗时: {end_time - start_time:.3f} 秒")
        return gg

# 绑定新方法到类
MVSGSNetEval.infer_cv = infer_cv

if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
    parser.add_argument('--valid_iters', type=int, default=16, help='number of flow-field updates during forward pass')

    # Architecture choices
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
    parser.add_argument('--depth_max', type=float, default=5, help="width of the correlation pyramid")
    parser.add_argument('--train_2layer', default=True, help="")

    parser.add_argument('--restore_ckpt', default=f'./checkpoints/raftmvs_2layer.pth', help="restore checkpoint")

    #########################################################################
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