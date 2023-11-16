import numpy as np
import math
import matplotlib.pyplot as plt
from pylab import mpl

# 设置显示中文字体
mpl.rcParams["font.sans-serif"] = ["SimHei"]
# 设置正常显示符号
mpl.rcParams["axes.unicode_minus"] = False

T = 1  # 雷达扫描周期
N = int(100 / T)  # 总采样次数
X = np.matrix(np.zeros((4, N)))
R0 = 5000  # 初始径向距离设为5km
alpha0 = 30  # 初始方位角设为30°
Vt = 15  # 目标运动速度为15m/s
xt_0 = R0 * math.cos(np.deg2rad(alpha0))
yt_0 = R0 * math.sin(np.deg2rad(alpha0))
vxt_0 = Vt * math.cos(np.deg2rad(alpha0))
vyt_0 = Vt * math.sin(np.deg2rad(alpha0))
X[:, 0] = np.array([xt_0, vxt_0, yt_0, vyt_0]).reshape(-1, 1)  # 目标初始状态[x, vx, y, vy]
Z = np.matrix(np.zeros((1, N)))  # 目标观测向量初始化
delta_w = 1 * np.exp(-10)
Q = delta_w * np.diag([1, 1])  # 过程噪声方差
G = np.matrix([
    [0.5 * T * T, 0],
    [T, 0],
    [0, 0.5 * T * T],
    [0, T]
])  # 过程噪声驱动矩阵
R = np.deg2rad(0.5)  # 观测噪声方差
F = np.matrix([
    [1, T, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, T],
    [0, 0, 0, 1]
])  # 状态转移矩阵
x0 = 0  # 观测站的x位置
y0 = 0  # 观测站的y位置
Xstation = np.hstack((x0, y0))

w = np.sqrt(R) * np.random.randn(1, N)


# 计算距离的子函数
def Distance(x1, x2):
    if len(x2) <= 2:
        d = np.sqrt((x1[0] - x2[0]) ** 2 + (x1[2] - x2[1]) ** 2)
    else:
        d = np.sqrt((x1[0] - x2[0]) ** 2 + (x1[2] - x2[2]) ** 2)

    return float(d)


# 计算方位的子函数
def hfun(x1, x2):
    theta = math.atan2(x1[2, 0] - x2[1], x1[0, 0] - x2[0])
    return theta


for t in range(2, N + 1):
    X[:, t - 1] = F * X[:, t - 2] + G * np.sqrt(Q) * np.random.randn(2, 1)  # 产生目标真实轨迹

for t in range(1, N + 1):
    Z[:, t - 1] = hfun(X[:, t - 1], Xstation) + w[:, t - 1]

# EKF滤波部分
Xekf = np.matrix(np.zeros((4, N)))
Xekf[:, 0] = X[:, 0]  # EKF滤波状态初始化
P0 = np.eye(4)  # 协方差矩阵初始化
for i in range(2, N + 1):
    Xn = F * Xekf[:, i - 2]  # 预测
    P1 = F * P0 * F.T + G * Q * G.T  # 预测误差协方差
    dd = hfun(Xn, Xstation)  # 观测预测
    D = Distance(Xn, Xstation)
    # 求雅各比矩阵
    H = np.matrix([
        [((-1) * Xn[2, 0] - y0) / D, 0, (Xn[0, 0] - x0) / D, 0]
    ])  # 雅各比矩阵
    S = H * P1 * H.T + R
    # print(S)
    # print(np.dtype(S))
    K = P1 * H.T * np.linalg.inv(S)  # 卡尔曼增益
    Xekf[:, i - 1] = Xn + K * (Z[:, i - 1] - dd)  # 状态更新
    P0 = (np.eye(4) - K * H) * P1  # 滤波误差协方差更新

# 绘制结果图
plt.figure(1)
plt.plot(X[0, :], X[2, :], color='#009933', linewidth=1, marker='.', label='目标真实轨迹')
plt.plot(Xekf[0, :], Xekf[2, :], color='#cc0066', linewidth=1, marker='+', label='EKF滤波轨迹')
plt.xlabel('x方向位置/m', fontsize=12, labelpad=15)
plt.ylabel('y方向位置/m', fontsize=12, labelpad=15)
plt.grid()

handles, labels = plt.gca().get_legend_handles_labels()  # 解决重复标签问题
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys())
plt.title('运动目标真实轨迹与EKF滤波轨迹对比图', fontsize=12)

# 误差结果表示
Err = np.zeros(N)
for i in range(1, N + 1):
    Err[i - 1] = Distance(X[:, i - 1], Xekf[:, i - 1])

plt.figure(2)
time_step = np.arange(1, N + 1)
plt.plot(time_step, Err, color='#FF6666',
         linestyle='-', linewidth=1,
         markersize=3, marker='o', label='目标真实轨迹')
plt.xlabel('观测时间/s', fontsize=12, labelpad=15)
plt.ylabel('位置估计偏差/m', fontsize=12, labelpad=15)
plt.title('EKF滤波偏差图', fontsize=12)
plt.grid()

plt.show()
