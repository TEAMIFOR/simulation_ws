#! /usr/bin/env python
import roslib
import rospy
import cv2
import sys, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3

class ellipse_detection:

    def __init__(self):
        cv2.namedWindow('mywindow', cv2.WINDOW_NORMAL)
        #cv2.namedWindow('thresh', cv2.WINDOW_NORMAL)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber( "camera1/image_raw",Image,self.callback )
        self.mid_pub = rospy.Publisher( "mid", Vector3, queue_size=10 )

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        l_M = 0
        h_M = 300
        l_m = 50
        h_m = 100
        thresh = 190

        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mblur = cv2.GaussianBlur(imgray,(5,5),0)
        ret,thresh = cv2.threshold(mblur,thresh,255,cv2.THRESH_BINARY)
        res = cv2.medianBlur(thresh,3)
        contours,hierarchy = cv2.findContours(res,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if len(cnt) > 5: # at least 5 pts needed
                box = cv2.fitEllipse(cnt)
                (x,y),(MA,ma),angle=cv2.fitEllipse(cnt)
                if(h_M>MA>l_M and h_m>ma>l_m):
                    #print x,y
                    cv2.ellipse(img,box,(200,0,0), 2)

        #cv2.imshow('thresh', thresh)
        cv2.imshow('mywindow', img)
        cv2.waitKey(1)

        vector = Vector3()
        vector.x = 0
        vector.y = 0
        vector.z = 0

        try:
            vector.x = float(x)
            vector.y = float(y)
            vector.z = float(0)
        except UnboundLocalError:
            pass

        self.mid_pub.publish(vector)



def main(args):
    ed = ellipse_detection()
    rospy.init_node('ellipse_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ellipse_detection")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
