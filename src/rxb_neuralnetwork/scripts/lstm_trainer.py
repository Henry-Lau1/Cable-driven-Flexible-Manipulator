#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import tensorflow as tf
import numpy as np

class MinMaxScaler:
    def __init__(self, min_=None, max_=None):
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

class LSTM_Trainer:
    num_inputs = 2
    num_labels = 2
    batch_size = 32
    timesteps = 20      # 50个采样点，时间间隔为20ms
    learning_rate = 0.001
    epochs = 30

    def __init__(self):
        rospy.init_node('lstm_trainer', anonymous=True)

        try:
            self.model = tf.keras.models.load_model("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/models/lstm_model.keras")
            print("已加载之前训练的模型。")
        except IOError:
            print("没有找到已保存的模型，创建新模型。")
            self.model = tf.keras.Sequential([
                tf.keras.layers.Input(shape=(self.timesteps, self.num_inputs)),
                tf.keras.layers.LSTM(units=16, activation='tanh', return_sequences=False),
                tf.keras.layers.Dense(units=32, activation='relu', kernel_initializer=tf.keras.initializers.HeNormal()),
                tf.keras.layers.Dense(units=64, activation='relu', kernel_initializer=tf.keras.initializers.HeNormal()),
                tf.keras.layers.Dense(units=self.num_labels, activation=None)
            ])
            self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=self.learning_rate),
                               loss=tf.keras.losses.MeanSquaredError(),
                               metrics=[tf.keras.metrics.MeanAbsoluteError()])

        self.inputs, self.labels = [], []

        self.inputsScaler = MinMaxScaler(-15, 15)
        self.labelsScaler = MinMaxScaler(0, 200)

        rospy.Subscriber('neuralNetwork/corCableForce/model/train', Bool, self.train_callback, queue_size=1)

    def load_data(self, file_path):
        data_lines = []
        with open(file_path, 'r') as file:
            for line in file:
                values = [float(x) for x in line.strip().split()]
                if len(values) != self.num_inputs * self.timesteps + self.num_labels:
                    print("数据格式错误，跳过该行")
                    continue
                data_lines.append(values)
        return np.array(data_lines)

    def train_callback(self, msg):
        if(msg.data != True):
            return

        # 加载数据
        data = self.load_data("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/data/corCableForce/trainSet.txt")
        X = data[:, :self.num_inputs * self.timesteps]
        Y = data[:, self.num_inputs * self.timesteps:]        
        # 归一化
        X_norm = self.inputsScaler.transform(X)
        Y_norm = self.labelsScaler.transform(Y)
        # 适应模型
        X_norm = X_norm.reshape((-1, self.timesteps, self.num_inputs))
        print(f"训练样本数：{X_norm.shape[0]}")
        # 训练
        history = self.model.fit(X_norm, Y_norm, batch_size=self.batch_size, epochs=self.epochs, verbose=1)
        # 保存模型
        self.model.save("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_neuralnetwork/models/lstm_model.keras")
        print("模型已保存。")

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    trainer = LSTM_Trainer()
    trainer.spin()
