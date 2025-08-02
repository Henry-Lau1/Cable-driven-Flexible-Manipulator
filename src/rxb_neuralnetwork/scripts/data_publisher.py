#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

def talker():
    pub = rospy.Publisher('neuralNetwork/corCableForce/train', Float32MultiArray, queue_size=32)
    rospy.init_node('data_publisher', anonymous=True)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        # 模拟2维输入+1维标签
        data = Float32MultiArray()
        input_data = np.random.rand(2)
        label = (input_data[0] + input_data[1]) * 0.5  # 假设真实关系
        data.data = input_data.tolist() + [label]
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    talker()
