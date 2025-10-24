import cv2
import numpy as np
import glob

# 设置棋盘格内角点数（根据你的棋盘格调整）
pattern_size = (8, 6)  # 内角点数，不是格子数

# 生成归一化的世界坐标（不需要知道实际尺寸！）
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# 注意：这里使用任意单位，比如假设方格边长为"1单位"
# 实际物理尺寸不影响内参计算结果！

obj_points = []
img_points = []

# 检测所有图片的角点
images = glob.glob('calib_images/*.jpg')

print(f"找到 {len(images)} 张图片")
success_count = 0

for i, fname in enumerate(images):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    
    if ret:
        # 亚像素精确化
        corners_subpix = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        
        obj_points.append(objp)
        img_points.append(corners_subpix)
        success_count += 1
        
        print(f"图片 {i+1}: 成功检测到角点")
    else:
        print(f"图片 {i+1}: 角点检测失败")

print(f"\n成功处理 {success_count} 张图片")

if success_count >= 3:  # 至少需要3张图片
    # 相机标定
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None
    )
    
    print("\n" + "="*50)
    print("相机内参矩阵 K:")
    print(f"fx = {K[0,0]:.2f} (x轴焦距)")
    print(f"fy = {K[1,1]:.2f} (y轴焦距)") 
    print(f"cx = {K[0,2]:.2f} (主点x坐标)")
    print(f"cy = {K[1,2]:.2f} (主点y坐标)")
    print("\n畸变系数:", dist.ravel())
    
    # 计算重投影误差
    mean_error = 0
    for i in range(len(obj_points)):
        img_points_repro, _ = cv2.projectPoints(
            obj_points[i], rvecs[i], tvecs[i], K, dist
        )
        error = cv2.norm(img_points[i], img_points_repro, cv2.NORM_L2) / len(img_points_repro)
        mean_error += error
    
    print(f"平均重投影误差: {mean_error/len(obj_points):.3f} 像素")
    
else:
    print("有效图片数量不足，需要至少3张成功检测角点的图片")