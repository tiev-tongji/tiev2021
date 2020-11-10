# -*- coding: utf-8 -*-
import math
import numpy as np  # 导入数值计算模块
import matplotlib.pyplot as plt  # 导入绘图模块


f_x = open("x.txt", "r")
f_y = open("y.txt", "r")

x = f_x.readlines()
y = f_y.readlines()
for i in range(0, len(x)):
    x[i] = x[i].rstrip('\n')
    y[i] = y[i].rstrip('\n')
for i in range(0, len(x)):
    x[i] = float(x[i])
    y[i] = float(y[i])
print(len(x))
print(x)

plt.plot(x, y)
plt.show()
