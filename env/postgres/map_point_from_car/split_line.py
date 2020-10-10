import os
import argparse
import numpy as np

from geopy.distance import geodesic

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_path", default="input_path.txt")
    parser.add_argument("--output_path", default="output_path.txt")
    args = parser.parse_args()

    all_points = open(args.output_path,'w')
    file_input = open(args.input_path, 'r') 
    file_input.readline()
    _id = 0
    line_id = 1
    all_points.write("Id l_id Lon Lat heading curvature mode SpeedMode EventMode OppositeSideMode LaneNum LaneSeq LaneWidth\n")
    
    line = file_input.readline()
    line = line.replace("\n","")
    str_list = list(filter(None,line.split(" ")))
    _id = _id+1
    x = float(str_list[1])
    y = float(str_list[2])
    head = float(str_list[3])
    curv = float(str_list[4])
    mode = int(str_list[5])
    velo_mode = int(str_list[6])
    event_mode = int(str_list[7])
    OppositeSideMode = int(str_list[8])
    lanNum = int(str_list[9])
    lanSeq = int(str_list[10])
    lanWidth = float(str_list[11])
    all_points.write("{} {} {} {} {} {} {} {} {} {} {} {} {}\n".format(_id, line_id, x, y, head,curv,mode, velo_mode,event_mode, OppositeSideMode, lanNum,lanSeq,lanWidth))

    prev_x = x
    prev_y = y

    while True:
        line = file_input.readline()
        if not line:
            file_input.close()
            break
        line = line.replace("\n","")
        str_list = list(filter(None,line.split(" ")))
        _id = _id+1
        x = float(str_list[1])
        y = float(str_list[2])
        head = float(str_list[3])
        curv = float(str_list[4])
        mode = int(str_list[5])
        velo_mode = int(str_list[6])
        event_mode = int(str_list[7])
        OppositeSideMode = int(str_list[8])
        lanNum = int(str_list[9])
        lanSeq = int(str_list[10])
        lanWidth = float(str_list[11])
        

        if geodesic((prev_y,prev_x), (y,x)).m > 1.5:
            # gaojia : 1.5
            # jiugongge : 1
            line_id = line_id+1
        
        all_points.write("{} {} {} {} {} {} {} {} {} {} {} {} {}\n".format(_id, line_id, x, y, head,curv,mode, velo_mode,event_mode,OppositeSideMode, lanNum,lanSeq,lanWidth))
        prev_x = x
        prev_y = y

if __name__ == "__main__":
    main()
