import multiprocessing as mp
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import socket
import time
import math
import torch

HOST = "192.168.1.3"                    # The remote host
PORT = 30002                            # The same port as used by the server
print ("Starting Program")

a = str(0.1)                            # Set max acceleration value for UR16e
v=str(0.1)                              # Set max Velocity value of UR16e
lookahead_time = str(0.1)               #FOr PID
gain = str(300)                         #For PID
p_home = [0,-0.5,0.6]                   # x , y , z coordinates of end effector relative to base frame at home position
r_home = [1.6848,0.949,-1.533839]       # (roll, pitch, yaw rotation vector of end effector relative to base frame at home position
e_home = [0,1.665,-2.115]               # (roll, pitch, yaw) euler angles of end effector relative to base frame at home position
p_camera = [-0.2,-0.5,0.660]            # x , y , z coordinates of realsense camera relative to base frame at home position (must change if camera posiiton moved)
p_endeffector = [0,-0.5,0.6]            # x , y , z coordinates of end effector relative to base frame (Should not be changed unless you want to change pinhole point of endeffector rotation

###### Compute Euler angles to rotation vector angles  ########
def euler_to_rotVec(yaw, pitch, roll):
    Rmat = euler_to_rotMat(yaw, pitch, roll)
    theta = math.acos(((Rmat[0, 0] + Rmat[1, 1] + Rmat[2, 2]) - 1) / 2)
    sin_theta = math.sin(theta)
    if sin_theta == 0:
        rx, ry, rz = 0.0, 0.0, 0.0
    else:
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (Rmat[2, 1] - Rmat[1, 2]) * theta
        ry = multi * (Rmat[0, 2] - Rmat[2, 0]) * theta
        rz = multi * (Rmat[1, 0] - Rmat[0, 1]) * theta
    return str(rx), str(ry), str(rz)

##### Compute euler angles to rotation matrix #####
def euler_to_rotMat(yaw, pitch, roll):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]])
    Ry_pitch = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]])
    # R = RzRyRx
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat

##### Process to move Robot Arm #####
def process_robot(q):
    real_point=[]                               #Initilise real_point to empty array
    roll= e_home[0]                             #Set initial euler angles of robot to be home euler angles.
    pitch= e_home[1]
    yaw= e_home[2]
    rotvec = euler_to_rotVec(yaw,pitch,roll)    #Convert euler angles to rotation vector

    #Begin Slew to home position
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    time.sleep(0.5)
    print ("Set output 1 and 2 high")
    string1=("set_digital_out(1,True)" + "\n")
    s.send (string1.encode())
    time.sleep(1)
    string2=("set_digital_out(2,True)" + "\n")
    s.send (string2.encode())
    time.sleep(1)
    print("Starting slew to home position")
    string6=('movel(p[' + str(p_home[0]) +',' + str(p_home[1]) + ',' + str(p_home[2]) + ',' + rotvec[0] + ',' + rotvec[1] +',' + rotvec[2] + '],a=' + a + ',v=' + v + ')\n')
    s.send (string6.encode())
    time.sleep(5)
    print("Robot is ready to follow target")

    #Slew to target obtained from ATDT Process
    while True:
        if not q.empty():
            real_point=q.get()      #Obtain x,y,z coordinates of target from ATDT process

            #Calculating Euler angles of real point
            x_d= real_point[2] + p_camera[0] - p_endeffector[0]             #obtain x, y and z distances between point 1 and end effector
            y_d = p_camera[1] - real_point[0] - p_endeffector[1]
            z_d = p_camera[2] - p_endeffector[2] - real_point[1]
            theta= math.atan(z_d / x_d)-0.1                                 #obtain change in pitch angle: theta 
            alpha = math.atan(y_d / x_d)                                    #obtain change in yaw angle: alpha
            roll = float(roll)                                              #adjust roll, pitch and yaw to get reference from base frame. 
            pitch = float(pitch) - theta
            yaw = float(yaw) + alpha

            #Calculate Rotation vector from Euler angles
            rotvec = euler_to_rotVec(yaw,pitch,roll)  

            print ("Robot slewing to target")
            string6=('servoj(get_inverse_kin(p[' + str(forward) +',' + str(p_home[1]) + ',' + str(p_home[2]) + ',' + rotvec[0] + ',' + rotvec[1] +',' + rotvec[2] +'])'+ ',0,0,0.008,' + lookahead_time + gain + ')\n')
            s.send (string6.encode())
            time.sleep(0.5)
            print("done")

##### Begin Main Process #####
if __name__ == '__main__':
    #Initialise multiprocessing
    mp.set_start_method('spawn')                    #Setting multiprocessing method
    q = mp.Queue(maxsize=1)                         #Setting up communication queue with a max list size of 1. Whenever queue is updated, the 1 element inside is replaced.
    p = mp.Process(target=process_robot, args=(q,)) #Linking variable p to robot process.

    
    #Configure depth, color streams and image size
    pipeline = rs.pipeline()
    config = rs.config()

    #Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    #Configure Realsense stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)

    #Load Yolov5 model. Require internet connection.
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom
    time.sleep(5)
    print("waited 5s, please plug in Ethernet cable now.")
    time.sleep(10)

    #Starting Robot Arm slewing process
    p.start()

    #Loop: Image capture from Realsense -> Get target pixel coordinates from Yolo-> Convert to real world coordinates -> Send real point to Queue.
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            ## Uncomment this to see the rgb and depth image side by side
            ## Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            #depth_colormap_dim = depth_colormap.shape
            #color_colormap_dim = color_image.shape
            #images = np.hstack((color_image, depth_colormap))

            
            #Yolo model Predict target bounding box from color image
            results = model(color_image)
            results.pandas().xyxy[0]     
            #cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
            #cv2.imshow(results.imgs[0])
            #print(results.pandas().xyxy[0])

            #Obtain center point of bounding box and send to queue. If no targets, dont send anything to queue.
            try:
                df = results.pandas().xyxy[0].loc[results.pandas().xyxy[0]['name'] == 'person']
                x = (df['xmin'].iloc[0] + df['xmax'].iloc[0])/2
                y = (df['ymin'].iloc[0] + df['ymax'].iloc[0])/3
                xmin = df['xmin'].iloc[0]
                ymin = df['ymin'].iloc[0]
                xmax = df['xmax'].iloc[0]
                ymax = df['ymax'].iloc[0]
                point = [int(x), int(y)]        #point is the x,y pixel coordinates of target
                depth = depth_frame.as_depth_frame().get_distance(point[0], point[1]) #Obtain depth of point.

                #Convert pixel coordinates to real world coordinates.
                real_point = rs.rs2_deproject_pixel_to_point(intrin, [point[0], point[1]], depth) 
                print("Target Spotted at this coordinates:")
                print(real_point)
                q.put(real_point)               #Send real_point coordinates to queue.
                time.sleep(1)                  #Put a sleep timer for now. Can remove if confident. 

                #Show image
                color_image = cv2.rectangle(color_image, (xmin, ymax), (xmax, ymin), (0, 0, 255), 2)
                cv2.imshow('image', color_image)
            except:
                cv2.imshow('image', color_image)
                pass

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()





