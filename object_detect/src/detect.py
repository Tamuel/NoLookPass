#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from no_look_pass_robot.msg import bounding


import cv2 # OpenCV module
from cv_bridge import CvBridge, CvBridgeError
import imutils

import numpy as np


def bgr_to_rgb(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def get_hist(image):
    hist = cv2.calcHist([image], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
    return cv2.normalize(hist).flatten()


master_img_path = "/home/turtlebot/Pictures/master.png"
master_img = cv2.imread(master_img_path)
master_img = bgr_to_rgb(master_img)
master_hist = get_hist(master_img)

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()


rospy.init_node('object_detection')

# Publish
detect_pub = rospy.Publisher('/bounding', bounding, queue_size=10)





def crop(image, rect):
    x = rect[0]
    y = rect[1]
    w = rect[2]
    h = rect[3]
    return image[y:y+h, x:x+w, :]


def detectCallBack(msg):
    pub_msg = bounding()
    sim_list = []

    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    image = imutils.resize(cv_image, width=min(400, cv_image.shape[1]))

    # detect people in the image
    (rects, weights) = hog.detectMultiScale(image, winStride=(4,4), padding=(8,8), scale=1.05)


    # rects = [rect.tolist() for rect in rects]
    weights = [ weight for weight in weights]

    if(len(rects)>0):
        # get index based on confidence of detection
        index = sorted(range(len(weights)), key=lambda k: weights[k], reverse=True)

        # sort 'rects' based on 'index' list
        rects = [rects[i] for i in index]

        # maximum length of 'rects' list is 3
        if(len(rects) > 2):
            rects = rects[:3]
        
        # crop
        for rect in rects:
            img = bgr_to_rgb(crop(image, rect))
            if(type(img) == type(None)):
                print("You FUcked UP")
                print(rects)
                continue
            #print("len of rect: ", len(rects))
            #print(img)
            #print("img shape:", img.shape)
            hist = get_hist(img)
            sim = cv2.compareHist(master_hist, hist, cv2.cv.CV_COMP_BHATTACHARYYA)
            sim_list.append(sim)
        
        # get index based on similarity(CHISQR)
        index = sorted(range(len(sim_list)), key=lambda k: sim_list[k])
        
        try:    
            # sort 'rects' based on 'index' list
            rects = [rects[i] for i in index]

            # change to rgb
            # gethist
            # get similarity
            # sort
            # publish most similar one

            #print(">>>>>>>>>>>>>>>")
            #print("This is weight of master")
            #print(weights)
            #print(">>>>>>>>>>>>>>>")
            
            # find bounding box of best 'master' detection 
            #print(">>>>>>>>>>>>>>>")
            #print("This is rects  of master")
            #print rects[0]
            #print(">>>>>>>>>>>>>>>")
            print("This is sim")
            print(sim_list[index[0]])
            #print(">>>>>>>>>>>>>>>")
            
            topx = rects[0][0]
            topy = rects[0][1]
            topw = rects[0][2]
            toph = rects[0][3]
            
            cv2.rectangle(image, (topx, topy), (topx+topw, topy+toph), (0, 255, 0), 2)

            pub_msg.x = float(topx)
            pub_msg.y = float(topy)
            pub_msg.w = float(topw)
            pub_msg.h = float(toph)

            detect_pub.publish(pub_msg)
            print("!!!!!!!!!!!!!!PUBLISH!!!!!!!!!!!!!!")

            rects = np.delete(rects, 0, axis=0)

            #if(len(rects)>0):
                # draw the original bounding boxes
            #    for (x, y, w, h) in rects:
            #        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)
        except IndexError:
            print("You Fucked UP >>'IndexError'")
            pass


    #cv2.imshow("window", image)
    #cv2.waitKey(3)
    #detect_pub.publish(cv_bridge.cv2_to_imgmsg(image))#, encoding="passthrough"))


if __name__=='__main__':

    rate = rospy.Rate(3)
    img_sub = rospy.Subscriber('/camera/rgb/image_color', Image, detectCallBack, queue_size=100)

    while not rospy.is_shutdown():
        # Subscribe    
        rate.sleep()
        #rospy.spin()

