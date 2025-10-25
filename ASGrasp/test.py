import numpy as np
import torch

def custom_inverse(matrix):
    try:
        # 使用torch.linalg.inv计算逆矩阵
        inverse_matrix = torch.linalg.inv(matrix)
        return inverse_matrix
    except RuntimeError as e:
        # 如果矩阵不可逆，捕获异常并返回None
        if "Singular matrix" in str(e):
            return None
        else:
            raise
 
 
# 创建一个随机的3x3矩阵，并假设设备支持CUDA
matrix = torch.rand(3, 3, device='cuda')
 
# 调用自定义求逆函数
inverse = custom_inverse(matrix)
 
# 输出结果
if inverse is not None:
    print("Original Matrix:\n", matrix.cpu().numpy())  # 将矩阵转移到CPU并转换为NumPy数组以便打印
    print("Inverse Matrix:\n", inverse.cpu().numpy())  # 同上
 
    # 验证逆矩阵是否正确
    identity_matrix = torch.mm(matrix, inverse)
    print("Identity Matrix:\n", identity_matrix.cpu().numpy())  # 同上
else:
    print("The matrix is singular and cannot be inverted.")