import os
import argparse
import numpy as np

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_path", default="input_path.txt")
    parser.add_argument("--output_path", default="output_path.txt")
    args = parser.parse_args()
    file_image = open(args.input_path, 'r') 
    file_output = open(args.output_path, 'w') 
    line = file_image.readline()
    file_output.write("Id Lon Lat heading curvature mode SpeedMode EventMode OppositeSideMode LaneNum LaneSeq LaneWidth\n")
    while True:
        line = file_image.readline()
        if not line:
            break
        line = line.replace('\n', '')
        str_list = list(filter(None,line.split(" ")))
        print(str_list)
        p_id = int(str_list[0])
        l_id = int(str_list[1])
        lon = float(str_list[2])
        lat = float(str_list[3])
        heading = float(str_list[4])
        curv = float(str_list[5])
        mode = int(str_list[6])
        speedmode = int(str_list[7])
        eventmode = int(str_list[8])
        OppositeSideMode = int(str_list[9])
        lanenum = int(str_list[10])
        laneseq = int(str_list[11])
        lanewidth = float(str_list[12])
        file_output.write("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(p_id,lon,lat,heading,curv,mode,speedmode,eventmode,OppositeSideMode,lanenum,laneseq,lanewidth))

if __name__ == "__main__":
    main()
