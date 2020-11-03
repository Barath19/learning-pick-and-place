from typing import Dict, List

import rospy  # pylint: disable=E0401
import sys

from bin_picking.srv import GetOrthographicImages  # pylint: disable=E0401, E0611

import utils.path_fix_ros  # pylint: disable=W0611

#from cv_bridge import CvBridge  # pylint: disable=E0611

from orthographical import OrthographicImage

def imgmsg_to_cv2(self, img_msg, desired_encoding = "passthrough"):
        """
        Convert a sensor_msgs::Image message to an OpenCV :cpp:type:`cv::Mat`.

        :param img_msg:   A :cpp:type:`sensor_msgs::Image` message
        :param desired_encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype: :cpp:type:`cv::Mat`
        :raises CvBridgeError: when conversion is not possible.

        If desired_encoding is ``"passthrough"``, then the returned image has the same format as img_msg.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns an OpenCV :cpp:type:`cv::Mat` message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.

        If the image only has one channel, the shape has size 2 (width and height)
        """
        import numpy as np
        dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        if n_channels == 1:
            im = np.ndarray(shape=(img_msg.height, img_msg.width),
                           dtype=dtype, buffer=img_msg.data)
        else:
            im = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels),
                           dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            im = im.byteswap().newbyteorder()

        if desired_encoding == "passthrough":
            return im
        
        # This should never be run as we always use the default encoding option
        return im

        # The cvtColor2 func is a wrapper function (cvtColor2Wrap) defined in module.cpp 
        # That wrapper function calls cvtColor() in cv_bridge.cpp, which in turn calls the
        # native OpenCV cv::cvtColor() function which is also made available to Python.
          
        #from cv_bridge.boost.cv_bridge_boost import cvtColor2

        #try:
        #    res = cvtColor2(im, img_msg.encoding, desired_encoding)
        #except RuntimeError as e:
        #    raise CvBridgeError(e)
        #return res

class Camera:
    def __init__(self, camera_suffixes: List[str]):
        self.suffixes = camera_suffixes
        #self.bridge = CvBridge()

        self.ensenso_suffixes = [s for s in self.suffixes if s in ['ed', 'er']]
        self.realsense_suffixes = [s for s in self.suffixes if s in ['rd', 'rc']]

        if len(self.ensenso_suffixes) + len(self.realsense_suffixes) != len(self.suffixes):
            raise Exception('Unknown camera suffix in {self.suffixes}!')

        if self.ensenso_suffixes:
            rospy.wait_for_service('ensenso/images')
            self.ensenso_service = rospy.ServiceProxy('ensenso/images', GetOrthographicImages)

        if self.realsense_suffixes:
            rospy.wait_for_service('realsense/images')
            self.realsense_service = rospy.ServiceProxy('realsense/images', GetOrthographicImages)

    def take_images(self) -> List[OrthographicImage]:
        def add_camera(service, suffixes: List[str], images: Dict[str, OrthographicImage]) -> None:
            result = service(suffixes)
            for i, img in enumerate(result.images):
                #mat = self.bridge.imgmsg_to_cv2(img.image, img.image.encoding)
                mat = imgmsg_to_cv2(img.image, img.image.encoding)
                images[suffixes[i]] = OrthographicImage(mat, img.pixel_size, img.min_depth, img.max_depth, img.camera)

        images: Dict[str, OrthographicImage] = {}

        if self.ensenso_suffixes:
            add_camera(self.ensenso_service, self.ensenso_suffixes, images)

        if self.realsense_suffixes:
            add_camera(self.realsense_service, self.realsense_suffixes, images)
        return [images[s] for s in self.suffixes]