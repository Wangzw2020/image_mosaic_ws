# coding:utf-8
import os
import time
import numpy as np
from PIL import Image as P_Image
import cv2
import sys
import roslib
import rospy
import threading

sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

image_topic_1 = "/cam_1/usb_cam/image_raw"
image_topic_2 = "/cam_2/usb_cam/image_raw"
image_topic_3 = "/cam_3/usb_cam/image_raw"
image_topic_4 = "/cam_4/usb_cam/image_raw"

class StitchingImage:
    def __init__(self):
        self._sub_Image1 = rospy.Subscriber(image_topic_1, Image, self.callback_1)
        self._sub_Image2 = rospy.Subscriber(image_topic_2, Image, self.callback_2)
        self._sub_Image3 = rospy.Subscriber(image_topic_3, Image, self.callback_3)
        self._sub_Image4 = rospy.Subscriber(image_topic_4, Image, self.callback_4)

        self._pub_stitched_image = rospy.Publisher("/pub_image", Image, queue_size=1)
        self.rate = 30
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.callback_stitch)
        self.lock_1 = threading.Lock()
        self.lock_2 = threading.Lock()
        self.lock_3 = threading.Lock()
        self.lock_4 = threading.Lock()
        
        self.image1 = []
        self.image2 = []
        self.image3 = []
        self.image4 = []
        
    def callback_1(self, image):
        self.lock_1.acquire()
        img = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)

        if len(self.Image1) > 0:
            self.Image1.pop(0)
            self.Image1.append(img)
        else:
            self.Image1.append(img)
        self.lock_1.release()
    
    def callback_2(self, image):
        self.lock_2.acquire()
        img = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)

        if len(self.Image2) > 0:
            self.Image2.pop(0)
            self.Image2.append(img)
        else:
            self.Image2.append(img)
        self.lock_2.release()
    
    def callback_3(self, image):
        self.lock_3.acquire()
        img = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)

        if len(self.Image3) > 0:
            self.Image3.pop(0)
            self.Image3.append(img)
        else:
            self.Image3.append(img)
        self.lock_3.release()
        
    def callback_4(self, image):
        self.lock_4.acquire()
        img = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)

        if len(self.Image4) > 0:
            self.Image4.pop(0)
            self.Image4.append(img)
        else:
            self.Image4.append(img)
        self.lock_4.release()
    
    def callback_stitch(self, event):
        
        self.lock_1.acquire()
        if len(self.Image1) > 0:
            tmp_Image1 = self.Image1[0]
            self.lock_1.release()
        else:
            self.lock_1.release()
            return
        
        self.lock_2.acquire()
        if len(self.Image2) > 0:
            tmp_Image2 = self.Image2[0]
            self.lock_2.release()
        else:
            self.lock_2.release()
            return
            
        self.lock_3.acquire()
        if len(self.Image3) > 0:
            tmp_Image3 = self.Image3[0]
            self.lock_3.release()
        else:
            self.lock_3.release()
            return
            
        self.lock_4.acquire()
        if len(self.Image4) > 0:
            tmp_Image4 = self.Image4[0]
            self.lock_4.release()
        else:
            self.lock_4.release()
            return
        
        print("start stitching!")
        
        
        
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = 'base_link'
        image_temp.encoding = 'rgb8'
        image_temp.height = int(480 * scale_factor)
        image_temp.width = int(640 * scale_factor)
        image_temp.step = int(640 * 3 * scale_factor)
        image_temp.data = np.array(fused_image).tostring()
        image_temp.header = header
        self._pub_fusion_image.publish(image_temp)
        
        ed = time.time()
        time_cost = ed - st
        print ("fusion time cost: ", time_cost)
    
    def find_kps_feas(self, image):
        # 建立SIFT生成器
        descriptor = cv2.xfeatures2d.SIFT_create()
        # 检测SIFT特征点
        (kps, features) = descriptor.detectAndCompute(image, None)
        npkps = np.float32([kp.pt for kp in kps])
        # 返回特征点集
        return (kps, npkps, features)
        
    def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh):
        bf = cv2.BFMatcher()
        allMatches = bf.knnMatch(featuresB, featuresA, k=2)
        matches = []
        good = []
        for m, n in allMatches:
            if m.distance < ratio * n.distance:
                matches.append((m.trainIdx, m.queryIdx))
                good.append([m])

        if len(matches) > 4:
            ptsA = np.float32([kpsA[i] for (i, _) in matches])
            ptsB = np.float32([kpsB[i] for (_, i) in matches])
            (H, status) = cv2.findHomography(ptsB, ptsA, cv2.RANSAC, reprojThresh)

        return (good, H, status)
        

def YCrCb2RGB(input_im):
    device = torch.device("cuda:{}".format(args.gpu) if torch.cuda.is_available() else "cpu")
    im_flat = input_im.transpose(1, 3).transpose(1, 2).reshape(-1, 3)
    mat = torch.tensor(
        [[1.0, 1.0, 1.0], [1.403, -0.714, 0.0], [0.0, -0.344, 1.773]]
    ).to(device)
    bias = torch.tensor([0.0 / 255, -0.5, -0.5]).to(device)
    temp = (im_flat + bias).mm(mat).to(device)
    out = (
        temp.reshape(
            list(input_im.size())[0],
            list(input_im.size())[2],
            list(input_im.size())[3],
            3,
        )
        .transpose(1, 3)
        .transpose(2, 3)
    )
    return out

def RGB2YCrCb(input_im):
    device = torch.device("cuda:{}".format(args.gpu) if torch.cuda.is_available() else "cpu")
    
    im_flat = input_im.transpose(1, 3).transpose(1, 2).reshape(-1, 3)  # (nhw,c)
    R = im_flat[:, 0]
    G = im_flat[:, 1]
    B = im_flat[:, 2]
    Y = 0.299 * R + 0.587 * G + 0.114 * B
    Cr = (R - Y) * 0.713 + 0.5
    Cb = (B - Y) * 0.564 + 0.5
    Y = torch.unsqueeze(Y, 1)
    Cr = torch.unsqueeze(Cr, 1)
    Cb = torch.unsqueeze(Cb, 1)
    temp = torch.cat((Y, Cr, Cb), dim=1).to(device)
    out = (
        temp.reshape(
            list(input_im.size())[0],
            list(input_im.size())[2],
            list(input_im.size())[3],
            3,
        )
        .transpose(1, 3)
        .transpose(2, 3)
    )
    
    return out
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run SeAFusiuon with pytorch')
    parser.add_argument('--model_name', '-M', type=str, default='SeAFusion')
    parser.add_argument('--batch_size', '-B', type=int, default=1)
    parser.add_argument('--gpu', '-G', type=int, default=0)
    parser.add_argument('--num_workers', '-j', type=int, default=8)
    args = parser.parse_args()
    seg_model_path = './model/Fusion/model_final.pth'
    fusion_model_path = './model/Fusion/fusionmodel_final.pth'
    
    rospy.init_node('fusion_node')
    FusionImage()
    rospy.spin()
    
