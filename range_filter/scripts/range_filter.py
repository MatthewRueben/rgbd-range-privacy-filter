#!/usr/bin/env python
# Applies a privacy filter (i.e., blur or redaction) to an image based on depth values.
# 1) Thresholds the depth image.
# 2) Applies it to the RGB image as a mask.
# 3) Applies a privacy filter. 

import rospy

if __name__ == "__main__":
    rospy.init_node('range_filter')
    rospy.spin()
