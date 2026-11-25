import unittest

import numpy as np
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from hl_oak_perception.common import image_message, image_array


class ImageTests(unittest.TestCase):
    def test_ros_serialization_is_lossless_for_all_encodings(self):
        for encoding, data in (
            ('bgr8',np.arange(6*8*3,dtype=np.uint8).reshape(6,8,3)),
            ('mono8',np.array([[0,255],[255,0]],np.uint8)),
            ('16UC1',np.array([[0,1,256],[1234,25000,65535]],dtype='<u2'))):
            message=image_message(data,encoding,Header(frame_id='camera'))
            restored=deserialize_message(serialize_message(message),Image)
            np.testing.assert_array_equal(image_array(restored),data)
            self.assertEqual(restored.header.frame_id,'camera')

    def test_strided_rgb_is_copied_and_original_changes_do_not_change_message(self):
        source=np.arange(8*8*3,dtype=np.uint8).reshape(8,8,3)
        expected=source[::2,::2].copy()
        message=image_message(source[::2,::2],'bgr8',Header())
        source[:]=0
        np.testing.assert_array_equal(image_array(message),expected)

    def test_invalid_step_or_length_is_rejected(self):
        message=image_message(np.zeros((8,8),np.uint16),'16UC1',Header())
        message.step+=1
        with self.assertRaises(ValueError):
            image_array(message)


if __name__=='__main__':
    unittest.main()
