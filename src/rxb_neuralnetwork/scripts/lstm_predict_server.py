#!/usr/bin/env python3
import rospy
from rxb_msgs.srv import corCableForceNN, corCableForceNNResponse
import tensorflow as tf
import numpy as np

class MinMaxScaler:
    def __init__(self, min_, max_):
        self.min = min_
        self.max = max_

    def transform(self, data):
        return (data - self.min) / (self.max - self.min)

    def inverse_transform(self, data_norm):
        return data_norm * (self.max - self.min) + self.min

class LSTM_Predictor:
    num_inputs = 2
    timesteps = 20

    def __init__(self):
        self.model = tf.keras.models.load_model("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/models/lstm_model.keras")
        self.inputsScaler = MinMaxScaler(-15, 15)
        self.labelsScaler = MinMaxScaler(0, 200)

        self.service = rospy.Service('neuralNetwork/corCableForce/model/call', corCableForceNN, self.handle_predict)
        print("预测服务已启动")

    def handle_predict(self, req):
        joint_angles = np.array(req.jointAngle)
        act_cable_force = np.array(req.actCableForce)

        if joint_angles.shape[0] != self.num_inputs * self.timesteps:
            rospy.logerr("jointAngle长度错误，应为{}".format(self.num_inputs * self.timesteps))
            return corCableForceNNResponse([], 0.0)
        if act_cable_force.shape[0] != 2:
            rospy.logerr("actCableForce长度错误，应为2")
            return corCableForceNNResponse([], 0.0)

        # 归一化 + reshape
        X_norm = self.inputsScaler.transform(joint_angles).reshape((1, self.timesteps, self.num_inputs))

        # 预测
        y_norm = self.model.predict(X_norm)
        y_pred = self.labelsScaler.inverse_transform(y_norm[0])

        # 损失计算（这里MSE）
        mse_loss = np.mean((y_pred - act_cable_force)**2)

        # 响应
        resp = corCableForceNNResponse()
        resp.predCableForce = y_pred.tolist()
        resp.loss = mse_loss
        return resp

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("force_predict_server")
    predictor = LSTM_Predictor()
    predictor.spin()
