import pybullet as p
import math
import numpy as np
import pybullet_data
from PIL import Image

# ugurkan ates , october 2020.
#!TODO
# better solution should use glm unproject -> look pyglm pip packege
def write_pointcloud(filename,pointCloud,img_h,img_w,stepX,stepY):

    # Write header of .ply file
    fid = open(filename,'w')
    fid.write(str('ply\n'))
    fid.write(str('format ascii 1.0\n'))
    fid.write(str('element vertex %d\n'%(pointCloud.size/4)))
    fid.write(str('property float x\n'))
    fid.write(str('property float y\n'))
    fid.write(str('property float z\n'))
    fid.write(str('end_header\n'))

    # Write 3D points to .ply file
    for h in range(0, img_h, stepY):
        for w in range(0, img_w, stepX):
                fid.write(str(pointCloud[h][w][0]) + " " + str(pointCloud[h][w][1])+ " " + 
                 str(pointCloud[h][w][2]) + "\n")
    fid.close()
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane100.urdf")
cube = p.loadURDF("cube.urdf", [0, 0, 1])
viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[2, 1, 3],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 1, 0])

projectionMatrix = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)
width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    width=224, 
    height=224,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix)
imgW = width #int(width / 10)
imgH = height #int(height / 10)
#Dimg = Image.fromarray(depthImg, 'F')
depth_img_buffer = np.reshape(depthImg, [imgW, imgH])
pointCloud = np.empty([np.int(imgH/1), np.int(imgW/1), 4])
projectionMatrix = np.asarray(projectionMatrix).reshape([4,4],order='F')
viewMatrix = np.asarray(viewMatrix).reshape([4,4],order='F')
tran_pix_world = np.linalg.inv(np.matmul(projectionMatrix, viewMatrix))

for h in range(0, imgH):
    for w in range(0, imgW):
        x = (2*w - imgW)/imgW
        y = -(2*h - imgH)/imgH  # be careful！ deepth and its corresponding position
        z = 2*depth_img_buffer[h,w] - 1
        pixPos = np.asarray([x, y, z, 1])
        position = np.matmul(tran_pix_world, pixPos)

        pointCloud[np.int(h/1),np.int(w/1),:] = position / position[3]
        #f.write(str(pointCloud[h][w])+"\n")
        #f.write(str(position[0]) + " " + str(position[1]) + " " + str(depth) + "\n")
write_pointcloud("fromSideAngle3.ply",pointCloud,imgH,imgW,1,1)

while (1):
  p.setGravity(0, 0, -10)