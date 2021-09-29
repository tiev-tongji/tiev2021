import sys
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton

class vec2d:
    def __init__(self, x_, y_):
        self.x = x_
        self.y = y_

    def __add__(self, other_):
        return vec2d(self.x + other_.x, self.y + other_.y)

    def __sub__(self, other_):
        return vec2d(self.x - other_.x, self.y - other_.y)

    def __mul__(self, value_):
        return vec2d(value_ * self.x, value_ * self.y)

    def __rmul__(self, value_):
        return vec2d(value_ * self.x, value_ * self.y)

    def dot(self, other_):
        return self.x * other_.x + self.y * other_.y

    def cross(self, other_):
        return self.x * other_.y - self.y * other_.x

    def sqr_norm(self):
        return self.x * self.x + self.y*self.y

class point:
    def __init__(self, x_, y_, heading_, lane_num_, lane_seq_, lane_width_):
        self.x = x_
        self.y = y_
        self.heading = heading_
        self.lane_num = lane_num_
        self.lane_seq = lane_seq_
        self.lane_width = lane_width_

    def __str__(self) -> str:
        return "[x={x}, y={y}, heading={heading}]".format(x = self.x, y = self.y, heading = self.heading)

    def tangent(self):
        return vec2d(math.cos(self.heading), math.sin(self.heading))

    def normal(self):
        return vec2d(math.sin(self.heading), -math.cos(self.heading))

    def lateral_point(self, distance_):
        lateral_vec2d = vec2d(self.x, self.y) + distance_ * self.normal()
        return point(lateral_vec2d.x, lateral_vec2d.y, self.heading, 0, 0, 0)

if len(sys.argv) != 2:
    print("please run \"python lane_lane_gen.py [roadmap_file_path]\"")
    exit(0)
filename = sys.argv[1]
try:
    road_data = np.loadtxt(filename, skiprows=1, dtype=float)
except:
    data_fram = pd.read_csv(filename, skiprows=1, dtype=float)
    data_fram = data_fram.iloc[:,1:]
    road_data = data_fram.to_numpy()
    print(road_data.shape)
    

path = []
for p in road_data:
    path.append(point(p[3], p[4], p[5], p[11], p[12], p[13]))
all_lane_line = []
for p in path:
    lane_lane_point_of_refpoint = []
    if p.lane_num == 0:
        continue
    for i in range(int(p.lane_num)+1):
        lane_lane_point_of_refpoint.append(p.lateral_point(p.lane_width * (i - p.lane_seq + 0.5)))
    all_lane_line.append(lane_lane_point_of_refpoint)
# draw lane line
ori_x = ori_y = 0
def mouse_call_back(event):
    # print(event.name, event.button, event.x, event.y, event.xdata, event.ydata)
    global ori_x, ori_y
    try:
        axtemp = event.inaxes
        x_min, x_max = axtemp.get_xlim()
        y_min, y_max = axtemp.get_ylim()
        x_range = (x_max - x_min) / 10
        y_range = (y_max - y_min) / 10
        if event.name != 'motion_notify_event' and event.button == 'up':
            axtemp.set(xlim = (x_min + x_range, x_max - x_range), ylim = (y_min + y_range, y_max - y_range))
        elif event.name != 'motion_notify_event' and event.button == 'down':
            axtemp.set(xlim = (x_min - x_range, x_max + x_range), ylim = (y_min - y_range, y_max + y_range))
        elif event.name == 'motion_notify_event' and event.button == MouseButton.MIDDLE:
            changed_x = (ori_x - event.xdata) / 2
            changed_y = (ori_y - event.ydata) / 2
            axtemp.set(xlim = (x_min + changed_x, x_max + changed_x), ylim = (y_min + changed_y, y_max + changed_y))
        ori_x, ori_y = event.xdata, event.ydata
        fig.canvas.draw_idle()
    except:
        pass
path_utmxs = []
path_utmys = []
path_utmx = []
path_utmy = []
for i in range(len(path)-1):
    pre_p = path[i]
    next_p = path[i+1]
    path_utmx.append(pre_p.x)
    path_utmy.append(pre_p.y)
    if (vec2d(next_p.x, next_p.y) - vec2d(pre_p.x, pre_p.y)).sqr_norm() > 100:
        path_utmxs.append(path_utmx)
        path_utmys.append(path_utmy)
        path_utmx = []
        path_utmy = []
lane_utmx = []
lane_utmy = []
for p in all_lane_line:
    if len(p) == 0:
        continue
    for pp in p:
        lane_utmx.append(pp.x)
        lane_utmy.append(pp.y)
try:
    fig = plt.figure()
    fig.canvas.mpl_connect('scroll_event', mouse_call_back)
    fig.canvas.mpl_connect('button_press_event', mouse_call_back)
    fig.canvas.mpl_connect('button_release_event', mouse_call_back)
    fig.canvas.mpl_connect('motion_notify_event', mouse_call_back)
    ax = fig.add_subplot(111)
    ax.set(title = "Road Map", ylabel = "Utm-Y", xlabel = "Utm-X")
    for i in range(len(path_utmxs)):
        ax.plot(path_utmxs[i], path_utmys[i], linewidth=1, color='green')
    ax.scatter(lane_utmx, lane_utmy, linewidths=0.1, color='violet')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
except:
    print('Exception')