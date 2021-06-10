import cv2
import numpy as np
import glob

def singleCalib(imgsPath, camID):
    # 获取标定板角点的位置
    nx = 8 # 棋盘格宽度方向的角点数
    ny = 11 # 棋盘格长度方向的角点数
    unit_mm = 30 # 单个棋盘的边长
    objp = np.zeros((ny * nx, 3), np.float32)
    # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
    objp[:, :2] = np.mgrid[0:ny, 0:nx].T.reshape(-1, 2)*unit_mm
    obj_points = []  # 存储3D点
    img_points = []  # 存储2D点
    images = glob.glob( "%s\\cam%d\\*.jpg" % (imgsPath, camID))
    i = 0
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(image=gray, patternSize=(ny, nx))
        if ret:
            obj_points.append(objp)
            # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
            criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
            # 在原角点的基础上寻找亚像素角点
            corners2 = cv2.cornerSubPix(image=gray, corners=corners, winSize=(5, 5), zeroZone=(-1, -1), criteria=criteria)  

            if [corners2]:
                img_points.append(corners2)
            else:
                img_points.append(corners)
            i = i + 1
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objectPoints=obj_points, imagePoints=img_points, imageSize=size, 
        cameraMatrix=None, distCoeffs=None, flags=(cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3))
    return obj_points, img_points, mtx, dist
    

if __name__ == "__main__":
    dataPath = "D:\\OneDrive\\Disk\\2-dataset\\xie2021\\archives\\20210608\\imgs\\calib"
    obj_points1, img_points1, mtx1, dist1 = singleCalib(imgsPath=dataPath, camID=1)
    obj_points2, img_points2, mtx2, dist2 = singleCalib(imgsPath=dataPath, camID=2)
    obj_points3, img_points3, mtx3, dist3 = singleCalib(imgsPath=dataPath, camID=3)

    # 相机2、1
    etval21, _, _, _, _, R21, T21, E21, F21 = cv2.stereoCalibrate(
        objectPoints=obj_points2, 
        imagePoints1=img_points2, imagePoints2=img_points1, 
        cameraMatrix1=mtx2, distCoeffs1=dist2, 
        cameraMatrix2=mtx1, distCoeffs2=dist1, 
        imageSize=(4096, 3072), flags=cv2.CALIB_FIX_INTRINSIC)
    # 相机2、3
    etval23, _, _, _, _, R23, T23, E23, F23 = cv2.stereoCalibrate(
        objectPoints=obj_points2, 
        imagePoints1=img_points2, imagePoints2=img_points3, 
        cameraMatrix1=mtx2, distCoeffs1=dist2, 
        cameraMatrix2=mtx3, distCoeffs2=dist3, 
        imageSize=(4096, 3072), flags=cv2.CALIB_FIX_INTRINSIC)

    # 输出到文件
    with open("calib.txt", "w") as f:
        for mtx in [mtx1, mtx2, mtx3]:
            f.write("%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n" \
                % (mtx[0, 0], mtx[0, 1], mtx[0, 2], mtx[1, 0], mtx[1, 1], mtx[1, 2], mtx[2, 0], mtx[2, 1], mtx[2, 2]))
        
        for dist in [dist1, dist2, dist3]:
            f.write("%.5f, %.5f, %.5f, %.5f\n" \
                % (dist[0, 0], dist[0, 1], dist[0, 2], dist[0, 3]))            

        for R in [R21, R23]:
            f.write("%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n" \
                % (R[0, 0], R[0, 1], R[0, 2], R[1, 0], R[1, 1], R[1, 2], R[2, 0], R[2, 1], R[2, 2]))
        for T in [T21, T23]:
            f.write("%.5f, %.5f, %.5f\n" \
                % (T[0], T[1], T[2]))
