import cv2
from cv_bridge import CvBridge, CvBridgeError

class ObservationProcessor:
    def __init__(self):
        self.bridge = CvBridge()

    def process(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = gray.reshape((1, 480, 640))

        # cv2.imwrite('image.png', img) 

        return gray