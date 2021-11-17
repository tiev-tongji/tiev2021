import sys
import math
import copy
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
    print("load from txt: ", road_data.shape)
except:
    data_fram = pd.read_csv(filename, skiprows=1, dtype=float)
    data_fram = data_fram.iloc[:,1:]
    road_data = data_fram.to_numpy()
    print("lode from csv: ", road_data.shape)
    

path = []
ori_utm_x = road_data[0][3]
ori_utm_y = road_data[0][4]
for p in road_data:
    path.append(point(p[3] - ori_utm_x, p[4] - ori_utm_y, p[5], p[11], p[12], p[13]))
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
path_segments = []
path_seg = []
all_lane_lines = []
lane_lines = []
now_lane_num = -1
now_lane_seq = -1
for i in range(len(path)-1):
    pre_p = path[i]
    next_p = path[i+1]
    path_seg.append(pre_p)
    if pre_p.lane_num != now_lane_num or pre_p.lane_seq != now_lane_seq:
        now_lane_num = pre_p.lane_num
        now_lane_seq = pre_p.lane_seq
        if len(lane_lines) > 0:
            all_lane_lines.append(copy.deepcopy(lane_lines))
        lane_lines = []
        for j in range(int(pre_p.lane_num)+1):
            if pre_p.lane_num < 1: break
            lane_p = pre_p.lateral_point(pre_p.lane_width * (j - pre_p.lane_seq + 0.5))
            lane_lines.append([lane_p])
    for j in range(int(pre_p.lane_num)+1):
        if pre_p.lane_num < 1: break
        lane_p = pre_p.lateral_point(pre_p.lane_width * (j - pre_p.lane_seq + 0.5))
        lane_lines[j].append(lane_p)
    if (vec2d(next_p.x, next_p.y) - vec2d(pre_p.x, pre_p.y)).sqr_norm() > 100:
        path_segments.append(copy.deepcopy(path_seg))
        path_seg = []
        now_lane_num = -1
        now_lane_seq = -1
if len(path_seg) > 0:
    path_segments.append(path_seg)
if len(lane_lines) > 0:
    all_lane_lines.append(lane_lines)
try:
    fig = plt.figure()
    fig.canvas.mpl_connect('scroll_event', mouse_call_back)
    fig.canvas.mpl_connect('button_press_event', mouse_call_back)
    fig.canvas.mpl_connect('button_release_event', mouse_call_back)
    fig.canvas.mpl_connect('motion_notify_event', mouse_call_back)
    ax = fig.add_subplot(111)
    ax.set(title = "Road Map", ylabel = "Y", xlabel = "X")
    for seg in path_segments:
        path_xs = []
        path_ys = []
        for p in seg:
            path_xs.append(p.x)
            path_ys.append(p.y)
        ax.plot(path_xs, path_ys, linewidth=1, color='green')
    for lane_lines in all_lane_lines:
        for line in lane_lines:
            line_xs = []
            line_ys = []
            for p in line:
                line_xs.append(p.x)
                line_ys.append(p.y)
            ax.plot(line_xs, line_ys, linewidth=1, color='violet')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
except:
    print('Exception')