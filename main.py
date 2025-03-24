import numpy as np

class KalmanFilter1D:
    def __init__(self, init_value=0.0, q=0.01, r=0.1):
        self.x_hat = init_value  # 状态估计值
        self.P = 1.0            # 估计误差协方差
        self.Q = q              # 过程噪声
        self.R = r              # 测量噪声

    def update(self, measurement):
        # 预测步骤
        x_pred = self.x_hat
        P_pred = self.P + self.Q

        # 更新步骤
        K = P_pred / (P_pred + self.R)  # 卡尔曼增益
        self.x_hat = x_pred + K * (measurement - x_pred)
        self.P = (1 - K) * P_pred
        
        return self.x_hat

# 模拟含噪声的传感器数据（真实值假设为10.0）
true_value = 8.88
measurements = true_value + np.random.normal(0, 1, 10)  # 生成10个带噪声数据

# 初始化滤波器
kf = KalmanFilter1D(measurements[0],q=0.1,r=0.5)

# 打印表头
print("测量序号\t原始数据\t滤波结果")
print("----------------------------------")

# 执行滤波并打印结果
for i, z in enumerate(measurements):
    filtered = kf.update(z)
    print(f"{i+1}\t\t{z:.2f}\t\t{filtered:.2f}")