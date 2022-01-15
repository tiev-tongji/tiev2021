import cv2
import numpy as np
import fire
import math
import array

class Point(object):
    def __init__(self, x, y,cx,cy):
        self.x = x
        self.y = y
        self.cx=cx
        self.cy=cy
    def __lt__(self,other):
        if self.x >= 0 and other.x < 0:
            return -1
        if self.x == 0 and other.x == 0:
            if self.y > other.y:
                return -1
            elif self.y < other.y:
                return 1
            return 0
        det = (self.x - self.cx) * (other.y - self.cy) - (other.x - self.cx) * (self.y - self.cy)
        if det < 0:
            return 1
        if det > 0:
            return -1
        d1 = (self.x - self.cx) * (self.x - self.cx) + (self.y - self.cy) * (self.y - self.cy)
        d2 = (other.x - self.cx) * (other.x - self.cx) + (other.y - self.cy) * (other.y - self.cy)
        if d1 > d2:
            return -1
        elif d1 < d2:
            return 1
        return 0

def calculat_cover_area(rect1,rect2):
    cover_shape = cv2.rotatedRectangleIntersection(rect1, rect2)
    if cover_shape[0] == 1:
        sum_x = 0
        sum_y = 0
        point = []
        len_p = cover_shape[1].shape[0]
        for i in range(len_p):
            sum_x += cover_shape[1][i][0][0]
            sum_y += cover_shape[1][i][0][1]
        center_x = sum_x / len_p 
        center_y = sum_y / len_p
        for i in range(len_p):
            point.append(Point(cover_shape[1][i][0][0],cover_shape[1][i][0][1],center_x,center_y))
        point.sort()
        rect = np.full((len_p, 2), 0.0, dtype='float32')
        for i in range(len(point)):
            rect[i][0] = point[i].x
            rect[i][1] = point[i].y
        area_and = cv2.contourArea(rect)
    elif cover_shape[0] == 0:
        area_and=0
    elif cover_shape[0] == 2:
        area_and=min(rect1[1][0]*rect1[1][1],rect2[1][0]*rect2[1][1])
    area_or=(rect1[1][0]*rect1[1][1]+rect2[1][0]*rect2[1][1])-area_and
    return area_and,area_or

def get_one_result(detection,labels_path,data_type,object_type,threshold):
    metadeta=detection["metadata"]
    label_index=metadeta["image_idx"]
    label_name="%06d" % label_index + ".txt"
    label_path=labels_path / label_name
    with open(label_path, "r") as f:
        lines = f.readlines()
    obs_num=len(detection["name"])
    good_num=0
    gt_num=0
    dt_num=0
    sum_iou=0
    for i in range(obs_num):
        if detection["name"][i] == object_type:
            dt_num+=1
    for line in lines:
        obj=line.split(' ')
        ttpye=obj[0]
        iou=0.0
        if ttpye != object_type:
            continue
        gt_num+=1
        angs=float(obj[14])
        ang=angs*180/math.pi
        if data_type == 'apollo':
            rect1=((float(obj[13]),float(obj[11])),(float(obj[8]),float(obj[10])),ang)
        elif data_type == 'kitti':
            rect1=((float(obj[13]),-float(obj[11])),(float(obj[8]),float(obj[10])),ang)
        for i in range(obs_num):
            if detection["name"][i] != object_type:
                continue
            dt_location = detection["location"][i]
            dt_dimension = detection["dimensions"][i]
            dt_angs=detection["rotation_y"][i]
            dt_ang=dt_angs*180/math.pi
            if data_type== 'apollo':
                rect2=((dt_location[0],dt_location[1]) , (dt_dimension[1],dt_dimension[0]) , dt_ang)
            elif data_type == 'kitti':
                rect2=((dt_location[0],dt_location[1]) , (dt_dimension[0],dt_dimension[1]) , dt_ang)
            area_and,area_or=calculat_cover_area(rect1,rect2)
            if area_and > threshold:
                good_num+=1
                iou=area_and/area_or
                break
        sum_iou+=iou
    if   dt_num != 0 and gt_num == 0:
        return good_num/dt_num,-1,-1
    elif dt_num == 0 and gt_num != 0  :
        return -1,good_num/gt_num,sum_iou/gt_num
    elif dt_num == 0 and gt_num == 0 :
        return -1,-1,-1
    else:
        return good_num/dt_num,good_num/gt_num,sum_iou/gt_num

def get_det_result(detections,labels_path,data_type,object_type,threshold=0):
    sum_acc=0
    sum_recall=0
    sum_iou=0
    sum_det=len(detections)
    true_p=sum_det
    true_d=sum_det
    for i in range(sum_det):
        detection = detections[i]
        acc,recall,iou=get_one_result(detection,labels_path,data_type,object_type,threshold)
        if acc<0:
            true_d-=1
        else:
            sum_acc+=acc
        if recall<0:
            true_p-=1
        else:
            sum_recall+=recall
            sum_iou+=iou
    if   true_d==0 and true_p!=0:
        return -1,sum_recall/true_p,sum_iou/true_p
    elif true_d==0 and true_p==0:
        return -1,-1,-1
    elif true_d!=0 and true_p==0:
        return sum_acc/true_d,-1,-1
    elif true_d!=0 and true_p!=0:
        return sum_acc/true_d,sum_recall/true_p,sum_iou/true_p

def get_det_results(detections,labels_path,data_type):
    acc,recall,iou=get_det_result(detections,labels_path,data_type,'Car',1)
    print('--Car Result--\nAvg_Precision:',acc,'\nAvg_Recall:',recall,'\nAvg_IoU:',iou)

    acc,recall,iou=get_det_result(detections,labels_path,data_type,'Cyclist',0)
    print('--Cyclist Result--\nAvg_Precision:',acc,'\nAvg_Recall:',recall,'\nAvg_IoU:',iou)

    acc,recall,iou=get_det_result(detections,labels_path,data_type,'Pedestrian',0)
    print('--Pedestrian Result--\nAvg_Precision:',acc,'\nAvg_Recall:',recall,'\nAvg_IoU:',iou)

    #acc,recall,iou=get_det_result(detections,labels_path,data_type,'Truck',0)
    #print('--Truck Result--\nAvg_Precision:',acc,'\nAvg_Recall:',recall,'\nAvg_IoU:',iou)
    
if __name__ == '__main__':
    fire.Fire()
