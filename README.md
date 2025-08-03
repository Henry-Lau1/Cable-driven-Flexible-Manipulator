# Cable-driven-Flexible-Manipulator
一个绳驱分段联动机械臂的上位机控制系统

## 项目简介
1. 基于QT+RViz设计UI界面插件，实现机械臂系统的状态监视和控制可视化
2. 针对绳驱分段联动机械臂的结构特性，配置URDF及SRDF文件，继承MoveIt实现简单规控
3. 结合CAN、UART及TCP通信协议，实现关节角度编码器、拉力传感器及六维力传感器的采集同步
4. 设计段空间闭环控制器及关节空间力位混合控制控制器，实现段端点的精确控制
5. 对机械臂系统进行动力学建模，实现段端点的径向力感知及柔顺控制
6. 结合LSTM等时序神经网络，实现数据驱动的刚度建模及传感器减配

## 项目结构
```bash
Cable-driven-Flexible-Manipulator
├── src/                                                # 源代码文件夹
│   ├── data                                            # 存储实验数据和中间结果
│   ├── rviz_control_plugin                             # RViz自定义控制面板插件
│   ├── rxb_bringup                                     # 启动脚本包
│   ├── rxb_description                                 # 机械臂URDF模型与视觉展示资源
│   ├── rxb_forceSense                                  # 机械臂动力学建模及力感知模块
│   ├── rxb_hardware_interface                          # 主节点
│   ├── rxb_moveit_config                               # MoveIt运动规划配置文件
│   ├── rxb_msgs                                        # 自定义ROS消息类型
│   ├── rxb_neuralnetwork                               # 神经网络模型
│   ├── rxb_taskspace_closeloop                         # 段空间闭环
│   ├── serial_communication                            # 与下位机UART串口通信
│   ├── sriforcesensor                                  # 与六维力传感器TCP通信
├── .gitignore                                          # Git 忽略文件配置
├── License                                             # 许可证文件
├── README.md                                           # 项目说明文档
├── cable_driven_flexible_manipulator.workspace         # Qt生成ROS工作空间文件1
├── cable_driven_flexible_manipulator.workspace.user    # Qt生成ROS工作空间文件2
```

## 许可证
版权所有 (c) 2025 Henry Lau

保留所有权利。

本项目及其所有内容为原创作品，任何组织或个人未经作者书面许可，不得以任何形式复制、转载、传播、修改、发布或使用，包括但不限于商业、教学、研究、展示等场景。

任何未经授权的使用行为均属于侵权行为，作者将依法追究其法律责任。

