# 匀速直线运动模型
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
from pylab import mpl

# 设置显示中文字体
mpl.rcParams["font.sans-serif"] = ["Simhei"]
# 设置正常显示字号
mpl.rcParams["axes.unicode_minus"] = False


def CV_motion(x_t0, y_t0, Ts, Tcv, C1, V1):
    t = np.arange(1, Tcv + 1, Ts)
    print('时间序列长度为：', t.shape, t)
    C = C1 * np.ones(Tcv)
    V = V1 * np.ones(Tcv)
    x_t = np.zeros(Tcv)
    y_t = np.zeros(Tcv)

    for i in t:
        x_t[i - 1] = x_t0 + V1 * i * math.cos(math.radians(C1))
        y_t[i - 1] = y_t0 + V1 * i * math.sin(math.radians(C1))
    # print('The shape of x_t is :', x_t.shape, 'The shape of y_t is :', y_t.shape)

    return x_t, y_t, C, V


# 测试
def main():
    R0 = 5 * 1000
    alpha0 = 10
    x_t0 = R0 * math.cos(math.radians(alpha0))
    y_t0 = R0 * math.sin(math.radians(alpha0))
    Ts = 1
    Tcv = 100
    V1 = 15
    C1 = 60
    x_cv, y_cv, C, V = CV_motion(x_t0, y_t0, Ts, Tcv, C1, V1)
    print('The shape of x_cv is:', x_cv.shape)
    print('The shape of y_cv is:', y_cv.shape)

    # 画态势图
    x_major_locator = MultipleLocator(200)
    y_major_locator = MultipleLocator(200)
    ax = plt.gca()
    ax.xaxis.set_major_locator(x_major_locator)
    ax.yaxis.set_major_locator(y_major_locator)
    plt.axis('equal')
    plt.plot(x_cv, y_cv, color='r', marker='o', linestyle='dashed')
    plt.plot(x_t0, y_t0, color='blue', marker='o')
    plt.xlabel('运动目标横坐标/m')
    plt.ylabel('运动目标纵坐标/m')
    plt.title('匀速直线运动目标态势图')
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
