#!/usr/bin/env python

"""
This module handle the job of processing the images captured by 2D camera of stocks 
which are arrived at the shelf and identify their respective colours.

The image captured by the 2D camera is cropped into 12 images each containing
an indiviual package. Then QR code in these images are decode to identify the 
colour of the packages at each row and coloumn of the shelf
"""

import rospy
import cv2
import json

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode


class ImageProcessing(object):
    """
    Class to analyse the packages on the warehouse shelf using
    2D camera and decode the QR on package to identify the color
    and store it in a package colors dictionary for further uses
    """
    #Constructor
    def __init__(self):
        rospy.loginfo("Ready for Package Details Processing..........") 
        self._bridge = CvBridge()
        self._callback_counter = 0
        self._package_color_dic = {}
        self._all_packages_analysed = False
        self._image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image , self.callback)


    def callback(self ,data):
        """
        This is a callback function , it is called everytime when the 2D camera 
        publish its feed on the ROS topic "/eyrc/vb/camera_1/image_raw"
        :param data:feed from the 2D camera
        :return :None
        """
        if self._callback_counter == 0:
            try:
                cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
                # Resize the image to  800 x 800
                resized_image = cv2.resize(cv_image, (800, 800))
                x_start = 90  #starting x pixel
                y_start = 175  #starting y pixel
                width = 200    #window crop width
                height = 100   #window crop height

                # Iterate through each row and column of the shelf
                for i in range (0,4):
                    for j in range(0,3):
                        crop_image = resized_image[y_start:y_start+height , x_start:x_start+width]

                        #Resizing croped image for better clarity
                        final_image = cv2.resize(crop_image, (150, 150))
                        package_color = decode_qr(final_image)
                        package_name  = "pkgn" + str(i) + str(j)

                        #Add the package_name and package_color as key and value pair to the dictionary
                        self._package_color_dic[package_name] = package_color
                        x_start = x_start + width 

                    x_start = 90 #Reset to starting x pixel
                    y_start = y_start + height

                #Number of times callback function is called
                self._callback_counter = 1

                #All packages analysed 
                self._all_packages_analysed = True

                #Unsubscribe the Ros topic "/eyrc/vb/camera_1/image_raw" 
                self._image_sub.unregister()

            except CvBridgeError as e:
                rospy.logerr(e)

    def wait_for_process_to_complete(self):
        """This function wait until all the packages on the self is analysed"""
        while not self._all_packages_analysed:
            pass

    def get_packages_details(self):
        """Return the dictionary of package_name and package_color pair"""
        return self._package_color_dic


def decode_qr(arg_image):
    """
    This function decode the QR present on the package
    and return the respective color of the package
    :param arg_image: the package image with QR code
    :return: decoded data
    """
    qr_result = decode(arg_image)

    if (len( qr_result ) > 0):
        decoded_data = qr_result[0].data
    else:
        decoded_data = "NA"

    #Return the Decode data from QR 
    return decoded_data

def detect_packages():
    """
    This function analyse all the packages on the shelf and publish their details on a 
    ROS Topic
    :return: Dictionary of package_name and package_color pair
    """
    #Initialsie the image processing class
    img_process = ImageProcessing()

    #Wait till all packages are analaysed
    img_process.wait_for_process_to_complete()

    #Package dicitonary
    package_dic = img_process.get_packages_details()

    return package_dic
