#!/usr/bin/env python3
"crime prevention robot"
# Check Pytorch installation
from logging import debug
from mmcv import image
import torch, torchvision
print(torch.__version__, torch.cuda.is_available())
import math as m 
# Check MMDetection installation
import mmdet
print(mmdet.__version__)

# Check mmcv installation
from mmcv.ops import get_compiling_cuda_version, get_compiler_version
print(get_compiling_cuda_version())
print(get_compiler_version())

import os
import sys
import cv2
import numpy as np
import time

# TensorRT
# from torch2trt import torch2trt
from mmdet.apis import inference_detector, init_detector, show_result_pyplot
# from mmdet2trt.apis import create_wrap_detector
# from mmdet2trt import mmdet2trt
# from torch2trt_dynamic import TRTModule

# subscribing pointcloud
import ros_numpy
# ROS related imports
import rospy
from std_msgs.msg import String, Header, Float32MultiArray, Float32, Int32, Float64MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sort.sort import *
from darknet_ros_msgs.msg import BoundingBoxes
# NOTE: 
# CvBridge meet problems since we are using python3 env
# We can do the data transformation manually
from cv_bridge import CvBridge, CvBridgeError

from vision_msgs.msg import Detection2D, \
                            Detection2DArray, \
                            ObjectHypothesisWithPose, \
                            BoundingBox2D    

from mmdetection_ros.srv import *

from mmdet.models import build_detector
# matching
import numpy as np
import seaborn as sns
from sklearn.cluster import DBSCAN

# thread
import threading
from threading import Thread
import copy

import multiprocessing
#recursion depth exceed
sys.setrecursionlimit(10000)

# Choose to use a config and initialize the detector
# CONFIG_NAME = 'yolov3_d53_aihub.py'
CONFIG_NAME = 'retinanet_r50_fpn_1x_aihub.py'

CONFIG_PATH = os.path.join(os.path.dirname(sys.path[0]),'scripts', CONFIG_NAME)

# Setup a checkpoint file to load
# MODEL_NAME =  'epoch_142.pth'    # yolo : 142, RetinaNet : 13
MODEL_NAME =  'epoch_14.pth'    # yolo : 142, RetinaNet : 13

MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]),'scripts', MODEL_NAME)

CLASSES = ('bicycle', 'bus', 'car', 'carrier', 'cat', 'dog', 'motorcycle',
            'movable_signage', 'person', 'scooter', 'stroller', 'truck',
            'wheelchair', 'barricade', 'bench', 'bollard', 'chair', 'fire_hydrant',
            'kiosk', 'parking_meter', 'pole', 'power_controller', 'potted_plant',
                  'stop', 'table', 'traffic_light', 'traffic_sign', 'tree_trunk')


import math

# Save trt file 
TRT_NAME = 'Retinanet_640X480_2.trt'
class Detector:

    def __init__(self, model):
        # self.image_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        # self.object_pub = rospy.Publisher("~objects", Float32MultiArray, queue_size=1)
        # self.object_pub = rospy.Publisher("~objects", Detection2DArray, queue_size=1)
        #self.results_pub = rospy.Publisher("~results", Detection2DArray, queue_size=1)
        self.results_pub = rospy.Publisher("~results", Float64MultiArray,queue_size=1)
        self.results_behind_pub = rospy.Publisher('~results_behind', Float64MultiArray, queue_size=1)
        self.bridge = CvBridge()
        self.model = model
        print(type(self.model))
        self._last_msg = None
        self._msg_lock = threading.Lock()
        
        self._publish_rate = rospy.get_param('~publish_rate', 30)
        self._is_service = rospy.get_param('~is_service', False)
        self._visualization = rospy.get_param('~visualization', False)
        self.clock = 0 
        self.img_data = 0
        self.img_height = 0
        self.img_width = 0
        self.img_msg = 0
        self.img_behind_data = 0
        self.img_behind_height = 0
        self.img_behind_width = 0
        self.img_behind_msg = 0

        self.points = 0
        self.results = 0
        self.object_list = []
        self.object_list_behind=[]
        
        self.frame_count = 0
        
        # behind cam thread
        behind_thread = Thread(target=self.run2)
        behind_thread.daemon = True
        behind_thread.start()

        
    # calibration matrix     
    def projection(self,x,y,z) : 
  
 
        roll = -1.6090   # -1.69392
        pitch = -0.00428  # -0.00428
        yaw = -1.522        # 1.53333
        distance_x =  -0.2011   # -0.07
        distance_y = 0.0092  #  -0.00484
        distance_z = 0.3395 #  0.15394
        Rotation_x = np.array([[1,0,0],[0, m.cos(roll),-m.sin(roll)],[0,m.sin(roll),m.cos(roll)]])
        Rotation_y = np.array([[m.cos(pitch), 0, m.sin(pitch)],[0, 1, 0],[-m.sin(pitch),0, m.cos(pitch)]])
        Rotation_z = np.array([[m.cos(yaw),-m.sin(yaw),0],[m.sin(yaw),m.cos(yaw),0],[0,0,1]])
        Rotation = np.dot(Rotation_z,np.dot(Rotation_y,Rotation_x))
        translation = np.array([[distance_x,distance_y,distance_z]])
        extrinsic_matrix = np.r_[Rotation, translation].T

        intrinsic_matrix = np.array([[381.618, 0.0, 318.674],[0.0,380.98,240.666],[0, 0, 1]])
        point_3D = np.array([x,y,z,1])
        point_2D = np.dot(intrinsic_matrix, np.dot(extrinsic_matrix, point_3D))
        result_x = point_2D[0] / point_2D[2]
        result_y = point_2D[1] / point_2D[2]
        return result_x,result_y
    
    def generate_obj(self, result, id, msg):
        obj = Detection2D()
        obj.header = msg.header
        obj.source_img = msg
        result = result[0]
        obj.bbox.center.x = (result[0] + result[2]) / 2
        obj.bbox.center.y = (result[1] + result[3]) / 2
        obj.bbox.size_x = result[2] - result[0]
        obj.bbox.size_y = result[3] - result[1]

        
        obj_hypothesis = ObjectHypothesisWithPose()
        obj_hypothesis.id = str(id)
        obj_hypothesis.score = result[4]
        obj.results.append(obj_hypothesis)
        self.bbox_pub.publish(obj.bbox)
        return obj
    
    lidar_to_cam = np.array([0,-1,0,0,0,-1,1,0,0]).reshape(3,3)

    # Visualize results
    def vis(self, image_np, results, msg, model):
        # self.thread = Thread(target=self.vis(image_np), args=(1,9999))
        # self.thread.start()
        
        # if self._visualization:
            # NOTE: Hack the provided visualization function by mmdetection
            # Let's plot the result
        # print("start visualization")
        # show_result_pyplot(self.model, image_np, results, score_thr=0.3)
            # if hasattr(self.model, 'module'):
            #     m = self.model.module
        self.clock += 1
        a = time.time()
        self.model.show_result(
                        image_np,
                        results,
                        score_thr=0.4 ,
                        show=False,
                        win_name='result',
                        bbox_color=(72, 101, 241),
                        text_color=(72, 101, 241),
                        out_file = "/home/patrol2/dummy1/"+str('{:04d}'.format(self.clock))+".png")
        
        b = time.time()
        print(b-a)
        #############
        # img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        # image_out = Image()
        # try:
        #     image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
        # except CvBridgeError as e:
        #     print(e)
        # image_out.header = msg.header
        # image_out.data = debug_image.tostring()c
        # self.image_pub.publish(image_out)
        ##############
        # No use cv_bridge
        # image_out = msg
        #     # NOTE: Copy other fields from msg, modify the data field manually
        #     # (check the source code of cvbridge)
        # # image_out.data = debug_image
        # image_out.data = debug_image.tostring()

        # self.image_pub.publish(image_out)
        
    # t = Thread(target=vis, args=(self,image_np,results,msg), kwargs = {})
    # t.start()
    
    def countdown(self,count):
        while count > 0:
            list = []
            print("Count value", list)
            count -= 1
            list.append(count)
        return

    
    def gt(self,results):
        for cls_idx in range(len(results)):
            if not results[cls_idx].any(): # 비어있는 디텍션 결과 거르기
                continue
            cls_label_arr = np.tile(np.array([cls_idx]), (results[cls_idx].shape[0], 1))
            try:
                det = np.hstack((results[cls_idx], cls_label_arr))
                dets = np.concatenate((dets, det))
            except:
                dets = np.hstack((results[cls_idx], cls_label_arr))
        try:
            return dets
        except:
            return False




    def _image_callback(self, msg):
        rospy.logdebug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self.img_msg = msg
            self.img_data = msg.data
            self.img_height = msg.height
            self.img_width = msg.width
            self._msg_lock.release()

    
    def _image_behind_callback(self, msg):
        rospy.logdebug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self.img_behind_msg = msg
            self.img_behind_data = msg.data
            self.img_behind_height = msg.height
            self.img_behind_width = msg.width
            self._msg_lock.release()
  
    
    
    def _pc_callback(self, msg):
        rospy.logdebug("Get a PC")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            pc = ros_numpy.numpify(msg)
            self.points = np.zeros((pc.shape[0],4))
            self.points[:,0] = pc['x']
            self.points[:,1] = pc['y']
            self.points[:,2] = pc['z']
            self.points[:,3] = 1
            self._msg_lock.release()
   
    def _yolo_callback(self, msg):
       rospy.logdebug("Get a detections")
       if self._msg_lock.acquire(False):
           self._last_msg = msg
           self.yolo = msg
           self.detection_boxes = np.zeros((len(self.yolo.bounding_boxes),6))
           for i in range(len(self.yolo.bounding_boxes)) :
               self.detection_boxes[i][0] = self.yolo.bounding_boxes[i].xmin
               self.detection_boxes[i][1] = self.yolo.bounding_boxes[i].ymin
               self.detection_boxes[i][2] = self.yolo.bounding_boxes[i].xmax
               self.detection_boxes[i][3] = self.yolo.bounding_boxes[i].ymax
               self.detection_boxes[i][4] = self.yolo.bounding_boxes[i].probability
               self.detection_boxes[i][5] = CLASSES.index(self.yolo.bounding_boxes[i].Class)
           self._msg_lock.release()
        
         
        
    
    def service_handler(self, request):
        return self._image_callback(request.image)
    
    def vector_from_origin(self,x,origin):
        return np.array([x[0]-origin[0],x[1]-origin[1],x[2]-origin[2]])

    def length(self,y):
        return m.sqrt(y[0]**2+y[1]**2+y[2]**2)
    
    def find_moving_object(self, xyz_arrs,object_list):
        distance_threshold = 0.35
        origin = (0,0,0)
        if len(object_list) ==0:
#            return xyz_arrs
            #0405
            return np.concatenate((xyz_arrs,np.zeros((xyz_arrs.shape[0],1))),axis=1)
            ##

        object_list.reverse()
        new_xyz_arrs =np.concatenate((xyz_arrs,np.zeros((xyz_arrs.shape[0],1))),axis=1)
#        for xyz_arr_idx in range(len(xyz_arrs)):
#            frequency = []
#            for object in object_list:
#                object_idx = np.where(object[:,-1]==xyz_arrs[xyz_arr_idx][-1])[0]
#                if len(object_idx) != 0:
#                    distance = abs(self.length(self.vector_from_origin(object[object_idx[0]],origin)) - self.length(self.vector_from_origin(xyz_arrs[xyz_arr_idx],origin)))
#                    frequency.append(distance)
#                if len(frequency) > 2:
#                    break
#            if len(frequency) != 0:
#                distance = sum(frequency)/len(frequency)
#                if distance > distance_threshold:
#                    new_xyz_arrs[xyz_arr_idx] = np.append(xyz_arrs[xyz_arr_idx],1)
#                else:
#                    new_xyz_arrs[xyz_arr_idx] = np.append(xyz_arrs[xyz_arr_idx],0)
#            else:
#                pass
        ##0405
        for xyz_arr_idx in range(len(xyz_arrs)):
            for object_ in object_list:
                object_idx = np.where(object_[:,4]==xyz_arrs[xyz_arr_idx][4])[0]
                if len(object_idx) != 0:
                    distance = abs(object_[object_idx[0]][0] - xyz_arrs[xyz_arr_idx][0])
                    if distance > distance_threshold:
                        new_xyz_arrs[xyz_arr_idx] = np.append(xyz_arrs[xyz_arr_idx],1)

                    else:
                        new_xyz_arrs[xyz_arr_idx] = np.append(xyz_arrs[xyz_arr_idx],0)
                    break
        ##


        return new_xyz_arrs
        
        
    
    def plot_points(self, dets, scan, image_np):
        
        # dets : x1 y1 x2 y2 cls_idx track_id
        # 여러 변수들 선언
        #lidar_to_cam = np.array([0,-1,0,0,0,-1,1,0,0]).reshape(3,3)
        mask_= []
        projected_scan = np.zeros_like(scan[:,:2])  # 이미지에 투영된 pcd
        #cam_scan = np.transpose(np.dot(lidar_to_cam, np.transpose(scan[:,:3]))) # 카메라 좌표계에서의 pcd
        width_arr = dets[:, 2] - dets[:, 0]
        height_arr = dets[:, 3] - dets[:, 1]
        
        # # pcd를 이미지에 projection한 후 원하는 포인트만 저장하는 과정
        #projected_scan[:,0], projected_scan[:,1] = self.projection(cam_scan[:,0], cam_scan[:,1], cam_scan[:,2])
        
               
        projected_scan[:,0], projected_scan[:,1] = self.projection(scan[:,0], scan[:,1], scan[:,2])
        projected_scan[:,1] = projected_scan[:,1] + 35
        
        
        u1 = np.array(list(map(int,projected_scan[:,0].tolist())))
        v1 = np.array(list(map(int,projected_scan[:,1].tolist())))
        offset_v = 5
        offset_u = 5
        u1_tile = np.tile(u1, (dets.shape[0], 1)) ####################################
        v1_tile = np.tile(v1, (dets.shape[0], 1))
        
        
        u1_range = (u1_tile > (dets[:, 0] + (width_arr/offset_u)).reshape(dets.shape[0], 1))\
            * (u1_tile < (dets[:, 2] - (width_arr/offset_u)).reshape(dets.shape[0], 1))
        v1_range = (v1_tile > (dets[:, 1] + (height_arr/offset_v)+25).reshape(dets.shape[0], 1))\
            * (v1_tile < (dets[:, 3] - (height_arr/offset_v)+45).reshape(dets.shape[0], 1))
        
        # u1_range = (u1_tile > (dets[:, 0]).reshape(dets.shape[0], 1))\
        #     * (u1_tile < (dets[:, 2]).reshape(dets.shape[0], 1))
        # v1_range = (v1_tile > (dets[:, 1]).reshape(dets.shape[0], 1))\
        #     * (v1_tile < (dets[:, 3]).reshape(dets.shape[0], 1))
        
        
        
        mask_arr = (v1_range * u1_range !=False)
        xyz_arrs = np.empty((0,4))
        #DBSCAN을 통해 위에서 얻은 pcd를 clustering하는 과정
        
        before_clustered_points = np.empty((0,4))
        
        in_box_points = np.empty((0,3))
        
        for k in range(len(mask_arr)):
            # x1 y1 x2 y2 score cls_idx track_id
            track_id = dets[k,-1]
            cls_idx = dets[k,-2]
            #0405
            xmin,ymin,xmax,ymax = dets[k,:4]
            ##
            mask_ = np.where(mask_arr[k])[0].tolist()
            masked_scan = scan[mask_arr[k],:3]
            
            in_box_points = np.concatenate((in_box_points,masked_scan),axis=0)
#            
            model = DBSCAN(eps=0.15, min_samples=9)
            #model = DBSCAN(eps=0.5, min_samples=5)
            try:
                yhat = model.fit_predict(masked_scan)
                list_ = []
                list_index = []
                list_depth = []

                for i in range(10):
                    _index = np.where(yhat == i)[0]
                    _index = np.array(mask_)[_index]
                    list_.append(_index.tolist())
                    list_index.append(len(_index))
                    
                    ##
                    list_depth.append(abs(np.mean(scan[_index][:,0])))
                    ##

                list_ = np.array(list_,dtype=object)
                list_index = np.array(list_index)
                ##
                list_depth = np.array(list_depth)
                min_depth = np.where(list_depth==min(list_depth))
                if list_.size == 0:
                    continue
                x = np.mean(scan[list_[min_depth][0],0])
                y = np.mean(scan[list_[min_depth][0],1])
                z = np.mean(scan[list_[min_depth][0],2])

                ##

                #max_index = np.where(list_index == max(list_index))
                
                
                #before_clustered_points = np.concatenate((before_clustered_points,scan[list_[max_index][0]]),axis=0)
                
                
                #x = np.mean(scan[list_[max_index][0],0])
                #y = np.mean(scan[list_[max_index][0],1])
                #z = np.mean(scan[list_[max_index][0],2])
           

           
                        
            #0405
#            x = np.mean(masked_scan[:,0])
#            y = np.mean(masked_scan[:,1])
#            z = np.mean(masked_scan[:,2])
#            u,v = self.projection(x,y,z)
#            if math.isnan(u) or math.isnan(v):
#                continue
            ##

            #projected_scan = np.concatenate((u,v),axis=-1)            
            #masked_scan = masked_scan[(np.isnan(projected_scan[:,0])==0)*(np.isnan(projected_scan[:,1])==0)]
                



                #import pdb;pdb.set_trace()
                #print(f"x:{x},y:{y},z:{z}")
                #print(f"u:{u},v:{v}")

#                try
#                    xyz_arr = np.array((x, y, z, cls_idx, track_id)).reshape(1, -1)
#                    xyz_arrs = np.concatenate((xyz_arrs, xyz_arr))
#                except:
#                    xyz_arrs = np.array((x, y, z, cls_idx, track_id)).reshape(1, -1)
            ##0405
            except:
                continue
            try:
                xyz_arr = np.array((x, y, z, cls_idx, track_id,xmin,ymin,xmax,ymax)).reshape(1, -1)
                xyz_arrs = np.concatenate((xyz_arrs, xyz_arr))
                    ##
                    
            except:
                
                    #xyz_arrs = np.array((x, y, z, cls_idx, track_id)).reshape(1, -1)
                    
                    
                    ##0405
                xyz_arrs = np.array((x, y, z, cls_idx, track_id,xmin,ymin,xmax,ymax)).reshape(1, -1)
            
                                ##
        # try:
        #     np.save("/home/patrol2/mmdetection_ws/ros_in_box_points.npy", in_box_points)
        #     np.save("/home/patrol2/mmdetection_ws/ros_clustering_points.npy", before_clustered_points)
        # except:
        #     pass
        #self.visualize(projected_scan, dets, image_np)
        
        
        #in_box_projected_scan = np.zeros_like(in_box_points[:,:2])  # 이미지에 투영된 pcd
        
        #in_box_projected_scan[:,0], in_box_projected_scan[:,1] = self.projection(in_box_points[:,0], in_box_points[:,1], in_box_points[:,2])



##0406
        np.save(f"/home/patrol2/mmdetection_ws/in_box_3d_points/{str(self.frame_count).zfill(6)}.npy",in_box_points)
        #self.visualize(in_box_projected_scan,dets,image_np)
        
        return xyz_arrs
        
    
    
    
    
    def visualize_img(self,image):        
        cv2.imwrite(f"/home/patrol2/mmdetection_ws/raw_image_front/{str(self.frame_count).zfill(6)}.jpg",image)

    def visualize_img_behind(self,image):        
        cv2.imwrite(f"/home/patrol2/mmdetection_ws/raw_image_behind/{str(self.frame_count).zfill(6)}.jpg",image)



    def visualize(self, projected_scan, dets, image):
        
        #cv2.imwrite("/home/patrol2/mmdetection_ws/demo_image.jpg",image)
        copied_image = copy.deepcopy(image)
        for det in dets:
            detected_img = cv2.rectangle(copied_image,(int(det[0]),int(det[1])),(int(det[2]),int(det[3])),(255,0,0),2)
            detected_img = cv2.rectangle(image,(int(det[0]),int(det[1])),(int(det[2]),int(det[3])),(255,0,0),2)
            
            for idx in range(len(projected_scan[:,0])):
                if projected_scan[:,0][idx]>det[0] and projected_scan[:,0][idx]<det[2] and projected_scan[:,1][idx]>det[1] and projected_scan[:,1][idx]<det[3]:
                    cv2.circle(copied_image,(int(projected_scan[:,0][idx]),int(projected_scan[:,1][idx])),2,(255,255,0),-1)
        cv2.imwrite(f"/home/patrol2/mmdetection_ws/track_projected_pcd/{str(self.frame_count).zfill(6)}.jpg",copied_image)
        #cv2.imwrite(f"/home/patrol2/mmdetection_ws/{str(self.frame_count).zfill(6)}.jpg",detected_img)
        
        
        for idx in range(len(projected_scan[:,0])):
        #cv2.putText(image,str(track_id),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,3,(0, 0, 255),10)
            cv2.circle(image,(int(projected_scan[:,0][idx]),int(projected_scan[:,1][idx])),2,(255,255,0),-1)
        
        cv2.imwrite(f"/home/patrol2/mmdetection_ws/track_projected_pcd_in_detections/{str(self.frame_count).zfill(6)}.jpg",image)
    
    def visualize_moving_object(self, xyz_arrs_with_move, image_np):
        #move : 1 not move :0
        
        image_for_depth = copy.deepcopy(image_np)
        
        origin = (0,0,0)
        projected_scan = np.zeros_like(xyz_arrs_with_move[:,:2])  # 이미지에 투영된 pcd
        projected_scan[:,0], projected_scan[:,1] = self.projection(xyz_arrs_with_move[:,0], xyz_arrs_with_move[:,1], xyz_arrs_with_move[:,2])
        for idx in range(len(projected_scan[:,0])):
            #cv2.putText(image,str(track_id),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,3,(0, 0, 255),10)
            
            #cv2.circle(image_np,(int(projected_scan[:,0][idx]),int(projected_scan[:,1][idx])),5,(255,255,0),-1)
            #cv2.circle(image_for_depth,(int(projected_scan[:,0][idx]),int(projected_scan[:,1][idx])),5,(255,255,0),-1)
            
            #distance = abs(self.length(self.vector_from_origin(xyz_arrs_with_move[idx],origin)))
    
            #try:
                #move_or_not = xyz_arrs_with_move[idx][5]
                #cv2.putText(image_np,str(int(move_or_not)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(0, 0, 255),2)
                #cv2.putText(image_for_depth,str(np.round(distance,1)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 0, 255),2)
                #cv2.putText(image_for_depth,str(np.round(xyz_arrs_with_move[idx,0],2)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,1,(255, 0, 255),5)
            #0405
            move_or_not = xyz_arrs_with_move[idx][-1]
            
            for det in xyz_arrs_with_move:
                if int(move_or_not) == 1:
                    cv2.rectangle(image_np,(int(xyz_arrs_with_move[idx][-5]),int(xyz_arrs_with_move[idx][-4])),(int(xyz_arrs_with_move[idx][-3]),int(xyz_arrs_with_move[idx][-2])),(0,0,255),2)
                else:
                    cv2.rectangle(image_np,(int(xyz_arrs_with_move[idx][-5]),int(xyz_arrs_with_move[idx][-4])),(int(xyz_arrs_with_move[idx][-3]),int(xyz_arrs_with_move[idx][-2])),(255,0,0),2)
                cv2.rectangle(image_for_depth,(int(xyz_arrs_with_move[idx][-5]),int(xyz_arrs_with_move[idx][-4])),(int(xyz_arrs_with_move[idx][-3]),int(xyz_arrs_with_move[idx][-2])),(255,0,0),2)


            #


            #distance = abs(self.length(self.vector_from_origin(xyz_arrs_with_move[idx],origin)))
            #0405
            distance = xyz_arrs_with_move[idx][0]
            
            try:
                ## 0405

                ##
                
                
                try:
                    cv2.putText(image_np,str(int(move_or_not)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 255, 255),2)
                    cv2.putText(image_for_depth,str(np.round(distance,1)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 255, 255),2)
                except:

                    pass
                
                #cv2.putText(image_for_depth,str(np.round(distance,1)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 0, 255),2)
                #cv2.putText(image_for_depth,str(np.round(xyz_arrs_with_move[idx,0],2)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,1,(255, 0, 255),5)


            except:
                cv2.putText(image_np,str(0),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 255, 255),2)
                cv2.putText(image_for_depth,str(np.round(distance,1)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 255, 255),2)
                #cv2.putText(image_for_depth,str(np.round(xyz_arrs_with_move[idx,0],2)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,1,(255, 0, 255),5)
        

        #visualize
        #if self.frame_count%5 == 0:
        #    cv2.imshow('run1',image_np)
        #    cv2.waitKey(2000)
        #    cv2.destroyAllWindows()


        cv2.imwrite(f"/home/patrol2/mmdetection_ws/track_ids/{str(self.frame_count).zfill(6)}.jpg", image_np)
        cv2.imwrite(f"/home/patrol2/mmdetection_ws/track_depth/{str(self.frame_count).zfill(6)}.jpg", image_for_depth)
            
    def visualize_trackids(self,track_bbs_ids,image_np):
        
        
        for idx in range(len(track_bbs_ids)):
            xmin = track_bbs_ids[idx][0]
            ymin = track_bbs_ids[idx][1]
            xmax = track_bbs_ids[idx][2]
            ymax = track_bbs_ids[idx][3]
            track_id = track_bbs_ids[idx][5] 
            
            img = cv2.rectangle(image_np,(int(xmin),int(ymin)),(int(xmax),int(ymax)),(255,0,0),2)
        
            cv2.putText(image_np,str(int(track_id)),(int(xmin),int(ymax)),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,3,(0, 0, 255),10)
        cv2.imwrite(f"/home/patrol2/mmdetection_ws/track_/{str(self.frame_count).zfill(6)}.jpg", image_np)
    
    def visualize_trackids_behind(self,track_bbs_ids,image_np):
        
        
        for idx in range(len(track_bbs_ids)):
            xmin = track_bbs_ids[idx][0]
            ymin = track_bbs_ids[idx][1]
            xmax = track_bbs_ids[idx][2]
            ymax = track_bbs_ids[idx][3]
            track_id = track_bbs_ids[idx][5] 
            
            img = cv2.rectangle(image_np,(int(xmin),int(ymin)),(int(xmax),int(ymax)),(255,0,0),2)
        
            cv2.putText(image_np,str(int(track_id)),(int(xmin),int(ymax)),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,3,(0, 0, 255),10)
        cv2.imwrite(f"/home/patrol2/mmdetection_ws/track_behind/{str(self.frame_count).zfill(6)}.jpg", image_np)

    
    def detector(self, model, image_np) : 
        results = inference_detector(model, image_np)
        return results
                
    def detector_behind(self, model, image_behind_np) : 
        results = inference_detector(model, image_behind_np)
        return results_behind
    
    def run(self):
        # for threading
        dummy =[0,1,2,3,4,5,6,7,8,9] 
        self.count= 0
        image_np = []
        image_np_behind = []
        results = []    
        msg = 0
        model = None 
        before_inf_detector_st = time.time()
        
        # start TensorRT(mm2trt)
        # REALSENSE
        opt_shape_param=[
                            [
                                [1,3,480,640],      # min shape
                                [1,3,480,640],     # optimize shape
                                [1,3,480,640],    # max shape
                            ]
                        ]
       
        cfg_path = os.path.join(os.path.dirname(sys.path[0]),'scripts' ,CONFIG_NAME)
        weight_path = os.path.join(os.path.dirname(sys.path[0]), 'scripts',MODEL_NAME)
        save_model_path = os.path.join(os.path.dirname(sys.path[0]), 'scripts',TRT_NAME)
        
        max_workspace_size=1<<30 
        
        # Save trt_model
        # trt_model = mmdet2trt(cfg_path, weight_path, opt_shape_param=opt_shape_param, max_workspace_size=max_workspace_size, fp16_mode=True)
        # torch.save(trt_model.state_dict(), save_model_path)
        
        # # # Loading trt_model
        
        # trt_model = TRTModule()
        
        # trt_model.load_state_dict(torch.load(save_model_path))
            
        # trt_detector = create_wrap_detector(trt_model, cfg_path)
        
        mot_tracker = Sort()
        if not self._is_service:
            rospy.loginfo('RUNNING MMDETECTOR AS PUBLISHER NODE')
            image_sub = rospy.Subscriber("~/cam_1/color/image_raw", Image, self._image_callback, queue_size=1)
            # image_behind = rospy.Subscriber("~/cam_2/color/image_raw", Image, self._image_behind_callback, queue_size=1)    # change topic name ? 
            pointcloud_sub = rospy.Subscriber("/velodyne_points",PointCloud2, self._pc_callback, queue_size=1)
            detections_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self._yolo_callback, queue_size=1)
            
        else:
            rospy.loginfo('RUNNING MMDETECTOR AS SERVICE')
            rospy.loginfo('SETTING UP SRV')
            srv = rospy.Service('~image', mmdetSrv, self.service_handler)
        
        
        print("start inference")
        rate = rospy.Rate(self._publish_rate)
        while not self.count == 0 :
            t = Thread(target=self.vis, args=(self, image_np, results, msg, model), kwargs = {})
            t.start()
        
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue
            
            if msg is not None:
                self.frame_count +=1
                # resultsArray = Detection2DArray()
                resultsArray = Float64MultiArray()
                # try:
                #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # except CvBridgeError as e:
                #     print(e)
                # NOTE: This is a way using numpy to convert manually
                
                try:
                    im = np.frombuffer(self.img_data, dtype = np.uint8).reshape(self.img_height, self.img_width, -1)
                    # im_behind = np.frombuffer(self.img_data, dtype = np.uint8).reshape(self.img_height, self.img_width, -1)
                    # im_behind = np.frombuffer(self.img_behind_data, dtype = np.uint8).reshape(self.img_behind_height, self.img_behind_width, -1)
                
                except:
                    continue
                
                # image = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
                image_np = np.asarray(im)
                image_np = cv2.cvtColor(image_np,cv2.COLOR_BGR2RGB)
                                         
                # if realsense
                
                # if ZED camera
                # image_np = np.delete(image_np,3,2)
                # image_np = np.zeros((376,672,3))
                # Use the detector to do inference
                # NOTE: inference_detector() is able to receive both str and ndarray
                inf_detector_st = time.time()
                # result share same format as mmdetection
                # results = inference_detector(trt_detector, image_np)
                #results = inference_detector(self.model, image_np)
                print("rear")                
                # results_behind = inference_detector(self.model, image_np_behind)
                
                # detection_infront = Thread(target=detector, args=(self, image_np, model), kwargs = {})
                # s1 = Thread(target=inference_detector, args=(self.model,image_np), kwargs = {})
                # s2 = Thread(target=results_behind, args=(self.model,image_np_behind), kwargs = {})
                
                # detection_infront.start()
                # s1.start()
                # s2.start()
                # detection_infront.join()
                # detection_behind.join()
                # xmin,ymin,xmax,ymax,score,cls_id
                
                #dets = self.gt(results)
                self.yolo
                dets = self.detection_boxes

                self.visualize_img(image_np)
                if not type(dets) == np.ndarray:
                    continue
                
                # poin = np.load("/home/patrol2/mmdetection_ws/ros_points.npy")
                # projected_scan = np.zeros_like(poin[:,:2])  # 이미지에 투영된 pcd
                # projected_scan[:,0], projected_scan[:,1] = self.projection(poin[:,0],poin[:,1],poin[:,2])
                # for idx in range(len(projected_scan[:,0])):
                
                #     cv2.circle(image_np,(int(projected_scan[:,0][idx]),int(projected_scan[:,1][idx])),2,(255,255,0),-1)
                
                # cv2.imwrite(f"/home/patrol2/mmdetection_ws/{str(self.frame_count).zfill(6)}.jpg",image_np)
                
                detections = dets[:,:6]
                track_bbs_ids = mot_tracker.update(detections)
                print("End : tracker")
                
                # track_bbs_ids : (xmin,ymin,xmax,ymax,class_id,track_id)
                image_np_trackids = copy.deepcopy(image_np)
                image_np_moving_object = copy.deepcopy(image_np)
                
                
                # vis1
                self.visualize_trackids(track_bbs_ids,image_np_trackids)
                
                
                ## image_projected_points
                masked_points = self.points[np.where(self.points[:,0]>0)]
                
                #masked_points = self.points
                np.save(f"/home/patrol2/mmdetection_ws/raw_pcd/{str(self.frame_count).zfill(6)}.npy",masked_points)
                
                
                #projected_scan = np.zeros_like(masked_points[:,:2])  # 이미지에 투영된 pcd
                #projected_scan[:,0], projected_scan[:,1] = self.projection(masked_points[:,0],masked_points[:,1],masked_points[:,2])
                #point_in_image_list = []
                
                # inds = projected_scan[:,0]>0
                # inds = np.logical_and(inds, projected_scan[:,0]<1280)
                # inds = np.logical_and(inds, projected_scan[:,1]>0)
                # inds = np.logical_and(inds, projected_scan[:,1]<720)
                
                # for idx in range(len(projected_scan[:,0])):
                #     if projected_scan[idx,0] > 0 and projected_scan[idx,0] < 640 and projected_scan[idx,1] > 0 and projected_scan[idx,1] < 480:
                #         point_in_image_list.append(idx)
                # np.save("/home/patrol2/mmdetection_ws/ros_in_image_points.npy",masked_points[point_in_image_list])
                
                
                ##
                xyz_arr = self.plot_points(track_bbs_ids, masked_points, image_np)
                #xyz_arr = self.plot_points(track_bbs_ids, self.points, image_np)
                print("End : clustering")
                #points_masked = self.points[np.where(self.points[:,0]>0)]
                #np.save("/home/patrol2/mmdetection_ws/ros_points.npy",points_masked)
                #xyz_arr = self.plot_points(track_bbs_ids, points_masked,image_np)
               
                print("obj_list : ", len(self.object_list))
                #xyz_arr : x,y,z,cls_idx,track_id,xmin,ymin,xmax,ymax (9)
                xyz_arrs_with_move = self.find_moving_object(xyz_arr, self.object_list)
                #xyz_arrs_with_move : (x,y,z,cls_id,track_id,xmin,ymin,xmax,ymax,move_or_not)
                print("End : moving or not")
                # vis2
                
                self.visualize_moving_object(xyz_arrs_with_move,image_np_moving_object)

                self.object_list.append(xyz_arrs_with_move)
                if len(self.object_list) > 11:
                    self.object_list.pop(0)
                
                
                final_results = np.concatenate((xyz_arrs_with_move[:,:4],xyz_arrs_with_move[:,-1][...,np.newaxis]),axis=-1)
                
                    # if cls_idx ==0:
                    #     dets = np.hstack((results[cls_idx], cls_label_arr))
                    # else:
                    #     det = np.hstack((results[cls_idx], cls_label_arr))
                    #     dets = np.concatenate((dets, det))
                
                
                
                results_data = []
#                cls_count = 0
#                for cls_idx in range(len(final_results)):
#                    if not final_results[cls_idx].any():
#                        continue
#                    classwise_results = final_results[cls_idx].tolist()
#                    for i in range(len(classwise_results)):
#                        if i == 0:
#                            rs = classwise_results[i]
#                        else:
#                            rs += classwise_results[i]
#                    results_data.append(rs)
                for num_ in range(len(final_results)):
                    #results_data.append(final_results[num_].tolist())
                    results_data += final_results[num_].tolist()
                results_data += [int(len(results_data) / 5)]
                #results = results[1][:4].tolist()[0][:3]+[1.0]
                
                # resultsArray.data = [0.0]
                resultsArray.data = results_data
                #results[1][:4].tolist()[0][:3]+[1.0]
                
                # resultsArray.header = self.img_msg.header
                # resultsArray.detections.append(resultsArray)
                # results = np.array(results)
                
                self.results_pub.publish(resultsArray)
                print("End : publish")
                #print("results")
                self.count+=1
                dummy.insert(self.count,inf_detector_st)
                FPS = round(5/(dummy[self.count] - dummy[self.count - 5]),2)
                
                # objArray.detections = []
                # objArray.header = msg.header
                # object_count = 1
                inf_detector_end = time.time() - inf_detector_st ###################################
                print("FPS = %f" %(FPS))
                # for i in range(len(results)):
                #     if results[i].shape != (0, 5):
                #         object_count += 1
                #         objArray.detections.append(self.generate_obj(results[i], i, msg))
                #         min_x = results[i][0][0]
                #         min_y = results[i][0][1]
                #         max_x = results[i][0][2]
                #         max_y = results[i][0][3]
                        
                        
                # if not self._is_service:
                    # self.object_pub.publish(objArray)
                #    self.object_pub.publish(objArray)
                #else:
                #    rospy.loginfo('RESPONSING SERVICE')
                #    return mmdetSrvResponse(objArray)
                

            rate.sleep()

    def find_moving_object2d(self, track_bbs_ids,object_list):
        if len(object_list)==0:
            return np.concatenate((track_bbs_ids,np.zeros((track_bbs_ids.shape[0],1))),axis=1)
        object_list.reverse()
        new_xyz_arrs=np.concatenate((track_bbs_ids,np.zeros((track_bbs_ids.shape[0],1))),axis=1)
        size_threshold_upper=1.01
        size_threshold_lower = 0.99

        for track_bbs_id in range(len(track_bbs_ids)):
            for object_ in object_list:
                object_idx = np.where(object_[:,-2]==track_bbs_ids[track_bbs_id][-1])[0]##chgd0408
                if len(object_idx) !=0:
                    prev_size=self.find_size(object_[object_idx[0]][0],object_[object_idx[0]][1],object_[object_idx[0]][2],object_[object_idx[0]][3])

                    #prev_size = self.find_size(object_[object_idx[0]])


                    cur_size=self.find_size(track_bbs_ids[track_bbs_id][0],track_bbs_ids[track_bbs_id][1],track_bbs_ids[track_bbs_id][2],track_bbs_ids[track_bbs_id][3])
                    
                    #cur_size = self.find_size(track_bbs_ids[track_bbs_id])
                    import pdb;pdb.set_trace()
                    print(cur_size/prev_size)
                    if cur_size/prev_size > size_threshold_upper:
                        new_xyz_arrs[track_bbs_id] = np.append(track_bbs_ids[track_bbs_id],1)
                    elif cur_size/prev_size < size_threshold_upper and cur_size/prev_size > size_threshold_lower:
                        new_xyz_arrs[track_bbs_id] = np.append(track_bbs_ids[track_bbs_id],0)
                    else:
                        new_xyz_arrs[track_bbs_id] = np.append(track_bbs_ids[track_bbs_id],-1)
                    break
        return new_xyz_arrs
    def find_size(self, xmin,ymin,xmax,ymax):
        
        x_length = abs(xmax-xmin)
        y_length = abs(ymax-ymin)
        return x_length * y_length
    
    
    def visualize_moving_object_behind(self, xyz_arrs_with_move, image_np):
        #move : 1 not move :0
        projected_scan = xyz_arrs_with_move
        
        for idx in range(len(projected_scan[:,0])):
            #cv2.putText(image,str(track_id),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,3,(0, 0, 255),10)
            
            #cv2.circle(image_np,(int(projected_scan[:,0][idx]),int(projected_scan[:,1][idx])),5,(255,255,0),-1)
            #cv2.circle(image_for_depth,(int(projected_scan[:,0][idx]),int(projected_scan[:,1][idx])),5,(255,255,0),-1)
            
            #distance = abs(self.length(self.vector_from_origin(xyz_arrs_with_move[idx],origin)))
    
            #try:
                #move_or_not = xyz_arrs_with_move[idx][5]
                #cv2.putText(image_np,str(int(move_or_not)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(0, 0, 255),2)
                #cv2.putText(image_for_depth,str(np.round(distance,1)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 0, 255),2)
                #cv2.putText(image_for_depth,str(np.round(xyz_arrs_with_move[idx,0],2)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,1,(255, 0, 255),5)
            #0405
            move_or_not = xyz_arrs_with_move[idx][-1]
            
            for det in xyz_arrs_with_move:
                if int(move_or_not) == 1:
                    cv2.rectangle(image_np,(int(xyz_arrs_with_move[idx][0]),int(xyz_arrs_with_move[idx][1])),(int(xyz_arrs_with_move[idx][2]),int(xyz_arrs_with_move[idx][3])),(0,0,255),2)
                elif int(move_or_not) == 0:
                    cv2.rectangle(image_np,(int(xyz_arrs_with_move[idx][0]),int(xyz_arrs_with_move[idx][1])),(int(xyz_arrs_with_move[idx][2]),int(xyz_arrs_with_move[idx][3])),(255,0,0),2)
                else:
                    cv2.rectangle(image_np,(int(xyz_arrs_with_move[idx][0]),int(xyz_arrs_with_move[idx][1])),(int(xyz_arrs_with_move[idx][2]),int(xyz_arrs_with_move[idx][3])),(0,255,0),2)

            #


            #distance = abs(self.length(self.vector_from_origin(xyz_arrs_with_move[idx],origin)))
            #0405
        
            image_size = self.find_size(xyz_arrs_with_move[idx][0],xyz_arrs_with_move[idx][1],xyz_arrs_with_move[idx][2],xyz_arrs_with_move[idx][3])
            try:
                ## 0405

                cv2.putText(image_np,str(int(move_or_not)),(int(xyz_arrs_with_move[idx][0]),int(xyz_arrs_with_move[idx][1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 255, 255),2)
                
                #cv2.putText(image_np,str(int(image_size)),(int(xyz_arrs_with_move[idx][0]),int(xyz_arrs_with_move[idx][1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 255, 255),2)
                #cv2.putText(image_for_depth,str(np.round(distance,1)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 0, 255),2)
                #cv2.putText(image_for_depth,str(np.round(xyz_arrs_with_move[idx,0],2)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,1,(255, 0, 255),5)

            except:
                cv2.putText(image_np,str(0),(int(xyz_arrs_with_move[idx][0]),int(xyz_arrs_with_move[idx][1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 255, 255),2)
                
                
                #cv2.putText(image_np,str(int(image_size)),(int(xyz_arrs_with_move[idx][0]),int(xyz_arrs_with_move[idx][1])),cv2.FONT_HERSHEY_PLAIN,1,(255, 255, 255),2)
                #cv2.putText(image_for_depth,str(np.round(xyz_arrs_with_move[idx,0],2)),(int(projected_scan[idx,0]),int(projected_scan[idx,1])),cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,1,(255, 0, 255),5)

        #visualize
        #if self.frame_count%5 == 0:
        #    cv2.imshow('run1',image_np)
        #    cv2.waitKey(2000)
        #    cv2.destroyAllWindows()


        cv2.imwrite(f"/home/patrol2/mmdetection_ws/track_ids_behind/{str(self.frame_count).zfill(6)}.jpg", image_np)
    
    def run2(self) :
        self.count= 0
        image_np = []
        image_np_behind = []
        results = []
        msg = 0
        model = None
        before_inf_detector_st = time.time()
        mot_tracker_behind = Sort()

        # start TensorRT(mm2trt)
        # REALSENSE
        opt_shape_param=[
                            [
                                [1,3,480,640],      # min shape
                                [1,3,480,640],     # optimize shape
                                [1,3,480,640],    # max shape
                            ]
                        ]
        
        

        cfg_path = os.path.join(os.path.dirname(sys.path[0]),'scripts' ,CONFIG_NAME)
        weight_path = os.path.join(os.path.dirname(sys.path[0]), 'scripts',MODEL_NAME)
        save_model_path = os.path.join(os.path.dirname(sys.path[0]), 'scripts',TRT_NAME)

        max_workspace_size=1<<30
              
        if not self._is_service:
            rospy.loginfo('RUNNING MMDETECTOR AS PUBLISHER NODE')
            image_behind = rospy.Subscriber("~/cam_2/color/image_raw", Image, self._image_behind_callback, queue_size=1)
        else:
            rospy.loginfo('RUNNING MMDETECTOR AS SERVICE')
            rospy.loginfo('SETTING UP SRV')
            srv = rospy.Service('~image', mmdetSrv, self.service_handler)


        print("start inference")
        rate = rospy.Rate(self._publish_rate)
        
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue

            if msg is not None:
                self.frame_count +=1
                # resultsArray = Detection2DArray()
                resultsArray = Float64MultiArray()
                # try:
                #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # except CvBridgeError as e:
                #     print(e)
                # NOTE: This is a way using numpy to convert manually

                try:
                    # im = np.frombuffer(self.img_data, dtype = np.uint8).reshape(self.img_height, self.img_width, -1)
                    im_behind = np.frombuffer(self.img_behind_data, dtype = np.uint8).reshape(self.img_behind_height, self.img_behind_width, -1)
                    
                
                except:
                    continue
                           
                image_behind_np = np.asarray(im_behind)
                image_behind_np = cv2.cvtColor(image_behind_np,cv2.COLOR_BGR2RGB)
                results = inference_detector(self.model, image_behind_np)
                self.visualize_img_behind(image_behind_np)
                dets = self.gt(results)

                
                if not type(dets) == np.ndarray:
                    continue
                
               
                detections = dets[:,:6]
 
                
                track_bbs_ids = mot_tracker_behind.update(detections)
                
                image_behind_np_trackids = image_behind_np.copy()
                xyz_arrs_with_move = self.find_moving_object2d(track_bbs_ids,self.object_list_behind)
                self.object_list_behind.append(xyz_arrs_with_move)
                if len(self.object_list_behind)>11:
                    self.object_list_behind.pop(0)
                
                final_results = xyz_arrs_with_move
                self.visualize_moving_object_behind(final_results,image_behind_np)
                print("behind")
                results_data = []
#                cls_count = 0
#                for cls_idx in range(len(final_results)):
#                    if not final_results[cls_idx].any():
#                        continue
#                    classwise_results = final_results[cls_idx].tolist()
#                    for i in range(len(classwise_results)):
#                        if i == 0:
#                            rs = classwise_results[i]
#                        else:
#                            rs += classwise_results[i]
#                    results_data.append(rs)

                self.visualize_trackids_behind(track_bbs_ids,image_behind_np_trackids)


                final_results = np.concatenate((xyz_arrs_with_move[:,4:5],xyz_arrs_with_move[:,-1][...,np.newaxis]),axis=-1)
                for num_ in range(len(final_results)):
                    #results_data.append(final_results[num_].tolist())
                    results_data += final_results[num_].tolist()
                
                results_data += [int(len(results_data) / 5)]

                #results_data += [int(len(results_data) / 4)]
                #results = results[1][:4].tolist()[0][:3]+[1.0]
                resultsArray.data = results_data
                #results[1][:4].tolist()[0][:3]+[1.0]

                # resultsArray.header = self.img_msg.header
                # resultsArray.detections.append(resultsArray)
                # results = np.array(results)
                try:
                    self.results_behind_pub.publish(resultsArray)
                except:
                    import pdb; pdb.set_trace()



def main(args):
    rospy.init_node('mmdetector')
    model = init_detector(CONFIG_PATH, MODEL_PATH, device='cuda:0')
    obj = Detector(model)
    obj.run()
    #obj.run2()
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("ShutDown")
    
    # print("start")

    # cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)
    

