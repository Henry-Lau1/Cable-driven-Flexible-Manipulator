#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import tensorflow as tf
import numpy as np
from rxb_msgs.msg import neuralNetworkProcess

class MinMaxScaler:
    def __init__(self, min_ = None, max_ = None):
        # 对于实时神经网络训练，最小值和最大值需要预定义
        self.min = min_
        self.max = max_
        
    def transform(self, data):
        if self.min is None or self.max is None:
            raise Exception("Must fit scaler before calling transform")
        return (data - self.min) / (self.max - self.min)
        
    def inverse_transform(self, data_norm):
        if self.min is None or self.max is None:
            raise Exception("Must fit scaler before calling inverse_transform")
        return data_norm * (self.max - self.min) + self.min

class MLP_Trainer:
    # 测试
    # num_inputs = 2
    # num_labels = 1
    # 实际
    num_inputs = 8
    num_labels = 6

    batch_size = 32

    def __init__(self):
        # 初始化节点
        rospy.init_node('mlp_trainer', anonymous=True)

        # 如果有之前保存的模型文件，加载它；没有则重新创建
        try:
            self.model = tf.keras.models.load_model("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/models/mlp_model.keras")
            print("已加载之前训练的模型。")
        except IOError:
            print("没有找到已保存的模型，创建新模型。")
            # 自定义神经网络模型
            self.model = tf.keras.Sequential([
                tf.keras.layers.Input(shape=(self.num_inputs,)), 
                tf.keras.layers.Dense(units=32, activation=tf.nn.relu, 
                                    kernel_initializer=tf.keras.initializers.HeNormal()), 
                tf.keras.layers.Dense(units=16, activation=tf.nn.relu, 
                                    kernel_initializer=tf.keras.initializers.HeNormal(),
                                    kernel_regularizer=tf.keras.regularizers.l2(0.001)), 
                tf.keras.layers.Dense(units=self.num_labels, activation=None, 
                                    kernel_initializer=tf.keras.initializers.glorot_uniform())
            ])
            self.model.compile(optimizer=tf.keras.optimizers.Adam(), 
                            loss=tf.keras.losses.MeanSquaredError(), 
                            metrics=[tf.keras.metrics.MeanAbsoluteError()])
        
        # 成员变量
        self.inputs, self.labels = [], []           # 数据输入及输出
        self.batch_count = 0                        # batch数量
        
        # 测试
        # self.inputsScaler = MinMaxScaler(0, 1)      # 输入归一化
        # self.labelsScaler = MinMaxScaler(0, 1)      # 输出归一化
        # 实际
        self.inputsScaler = MinMaxScaler(-15, 15)      # 输入归一化
        self.labelsScaler = MinMaxScaler(0, 200)      # 输出归一化

        # ros通信
        rospy.Subscriber('neuralNetwork/corCableForce/train', Float32MultiArray, self.data_callback, queue_size=32)
        self.processPub = rospy.Publisher("neuralNetwork/corCableForce/process", neuralNetworkProcess, queue_size=32)

    def data_callback(self, msg):
        data = msg.data
        if len(data)!=self.num_inputs+self.num_labels:
            rospy.logwarn('Received data size mismatch.')
            return
        
        x = self.inputsScaler.transform(np.array(data[:self.num_inputs]))
        y = self.labelsScaler.transform(np.array(data[self.num_inputs:]))
        self.inputs.append(x)
        self.labels.append(y)

        if len(self.inputs) >= self.batch_size:
            self.batch_count += 1
            
            X_train = np.array(self.inputs)
            y_train = np.array(self.labels)
            history = self.model.fit(X_train, y_train, batch_size=self.batch_size, epochs=1, verbose=0)

            batch_count = self.batch_count
            y_pred = self.model.predict(X_train[-1].reshape(1, -1), verbose=0)[0]
            loss = history.history['loss'][0]
            y_act = y_train[-1]
            mae = history.history['mean_absolute_error'][0]

            print(f"====== Batch {batch_count} 完成 ======")
            print(f"最后一个样本：实际值 = {self.labelsScaler.inverse_transform(y_act[0])}, 预测值 = {self.labelsScaler.inverse_transform(y_pred[0])}")
            print(f"归一化后：Loss = {loss:.6f}, MAE = {mae:.6f}")
            print("==================================")

            processData = neuralNetworkProcess()
            processData.batch = batch_count
            processData.actualVal = self.labelsScaler.inverse_transform(y_act)
            processData.loss = loss
            processData.predictVal = self.labelsScaler.inverse_transform(y_pred)
            self.processPub.publish(processData)

            self.inputs, self.labels = [], []

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

        print("检测到 Ctrl+C，保存模型中...")
        trainer.model.save("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/models/mlp_model.keras")
        print("模型已保存，程序退出。")

if __name__ == '__main__':
    trainer = MLP_Trainer()
    trainer.spin()


