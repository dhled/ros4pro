from intera_interface.camera import Cameras as RealCameras
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospkg.rospack import RosPack
from os.path import join
from random import choice
import cv2, glob, rospy

class Camera(object):
    # This is an abstraction for simulated and real right_hand_camera
    def __init__(self):
        simulated = rospy.get_param("ros4pro/light_simulation", False)
        self._camera = None if simulated else RealCameras()
        self.bridge = CvBridge()
        self.rospack = RosPack()

        self._pub_picture = rospy.Publisher("/io/internal_camera/right_hand_camera/image_rect", Image, queue_size=1)

        # Load simulated images. Filename format: type_label1[_label2]_fileId.jpg
        self.images = {"duo": [], "straight": [], "rotations": []}
        path = join(self.rospack.get_path('ros4pro'), "assets", "camera_images")
        files = glob.glob(join(path, "*.jpg"))
        for file in files:
            params = file.split("/")[-1].split('.')[0].split("_")
            type = params[0]
            self.images[type].append({"path": file})
            
            for i in range(len(params)-2):
                image = cv2.imread(file)
                greyscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                image_msg = self.bridge.cv2_to_imgmsg(greyscale_image)
                self.images[type][-1]["msg"] = image_msg

    def shoot(self):
        if self._camera:
            self._camera.set_cognex_strobe(True)
            rospy.sleep(0.1)
            self._camera.set_cognex_strobe(False)
        else:
            # Pick a random simulated picture now
            type = choice(self.images.keys())
            image = choice(self.images[type])
            rospy.loginfo("Simulating camera image {}".format(image["path"]))
            self._pub_picture.publish(image["msg"])
        rospy.sleep(1)
