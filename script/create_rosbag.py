import os
from typing import Any, overload
import numpy as np

import rosbag 
import rospy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import argparse
import cv2 as cv
import pandas as pd

class ROSBag():
    def __init__(self):
        self.imu_data = None
        self.left_image = None
        self.right_image = None

    def parse_options(self, *args, **kwargs):
        parser = argparse.ArgumentParser()
        parser.add_argument("--left_images", "-l", type=str, required=True, help="Path to left image")
        parser.add_argument("--right_images", "-r", type=str, required=False, help="Path to right image")
        parser.add_argument("--imu_datas", "-i", type=str, required=False, help="Path to imu data")
        parser.add_argument("--save_path", "-s", type=str, required=True, help="Path to save bag")
        arg = parser.parse_args()

        self.imu_data = arg.imu_datas
        self.left_image = arg.left_images
        self.right_image = arg.right_images
        self.save_path = arg.save_path

        print("left_image path is %s" % self.left_image)
        print("save path is %s" % self.save_path)

    def ReadRGB(self, file_dir):
        "Here assume the name of image is the timestamp"
        file_names_left = sorted(os.listdir(file_dir))
        def get_timestamp(file_name):
            return np.float64(file_name[:-4])
        timestamps = map(get_timestamp, file_names_left)
        print("Total add %i images!" % (np.size(file_names_left)) )
        return file_names_left, list(timestamps)
    
    def ReadIMU(self, file_path, use_csv):
        '''return IMU data and timestamp of IMU'''
        timestamp = []
        imu_data = []
        index = 0
        if use_csv:
            df = pd.read_csv(filepath_or_buffer=file_path)
            for i in range(df.shape[0]):
                index += 1
                timestamp.append(df["#timestamp [ns]"][i])
                imu_data.append(df.iloc[i, 1:7].values)
        else:
            file = open(file_path, 'r')
            all = file.readlines()
            for f in all:
                index += 1
                line = f.rstrip('\n').split(' ') # here maybe ',' or ' '
                timestamp.append(line[0])
                imu_data.append(line[1:7])
        print("Total add %i imus!" % (index))
        return imu_data, timestamp

    def CreateBag(self):

        bag = rosbag.Bag(self.save_path, 'w')
        file_names_left, imgstamp_left = self.ReadRGB(self.left_image)
        if self.right_image != None:
            file_names_right, imgstamp_right = self.ReadRGB(self.right_image)
        # print(file_names_left)
        # print(timestamp)
        if self.imu_data != None:
            imu_data, imustamp = self.ReadIMU(self.imu_data, True) # used for csv file
        # print(imu_data)
        # print(timestamp)
        print('working!')

        # print(len(imu_data))

        for i in range(len(file_names_left)):
            img = CvBridge().cv2_to_imgmsg(cv.imread(self.left_image + file_names_left[i], cv.IMREAD_GRAYSCALE))
            img.header.frame_id = "camera"
            img.header.stamp = rospy.rostime.Time.from_sec(imgstamp_left[i] * 1e-9)
            img.encoding = "mono8"
            bag.write("/stereo_inertial_publisher/left/image_rect", img, img.header.stamp)

        if self.right_image != None:
            for i in range(len(file_names_right)):
                img = CvBridge().cv2_to_imgmsg(cv.imread(self.right_image + file_names_right[i], cv.IMREAD_GRAYSCALE))
                img.header.frame_id = "camera"
                img.header.stamp = rospy.rostime.Time.from_sec(imgstamp_right[i] * 1e-9)
                img.encoding = "mono8"
                bag.write("/stereo_inertial_publisher/right/image_rect", img, img.header.stamp)

        if self.imu_data != None:
            for i in range(0, len(imu_data)):
                # print(i)
                imu = Imu()
                angular_v = Vector3()
                linear_a = Vector3()
                angular_v.x = float(imu_data[i][0])
                angular_v.y = float(imu_data[i][1])
                angular_v.z = float(imu_data[i][2])
                linear_a.x = float(imu_data[i][3])
                linear_a.y = float(imu_data[i][4])
                linear_a.z = float(imu_data[i][5])
                imuStamp = rospy.rostime.Time.from_sec(imustamp[i] * 1e-9)  # according to the timestamp unit
                imu.header.stamp = imuStamp
                imu.angular_velocity = angular_v
                imu.linear_acceleration = linear_a

                bag.write("/camera/imu", imu, imuStamp)

        bag.close()

        print ("done!")

    def __call__(self, *args, **kwds):
        self.parse_options(args=args, kwargs=kwds)
        self.CreateBag()
        # self.ReadIMU(self.imu_data, True)

if __name__ == "__main__":
    # bag = rosbag.Bag("", 'w')
    # bag.write("/camera/imu", Image(), )
    bag = ROSBag()
    bag()
