import os
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir_path", default="sdfghj")
    args = parser.parse_args()
    
    file_write = open("./merge_all.csv",'w')
    file_write.write("Id Lon Lat heading curvature mode SpeedMode EventMode OppositeSideMode LaneNum LaneSeq LaneWidth\n")
    _id = 0;
    for maindir, _, file_name_list in os.walk(args.dir_path):
        for filename in file_name_list:
            apath = os.path.join(maindir, filename)
            print(apath)
            file_read = open(apath,'r')
            file_read.readline()
            while True:
                line = file_read.readline()
                if not line:
                    file_read.close()
                    break
                
                line = line.replace("\n","")
                str_list = list(filter(None,line.split(" ")))
                #_id = int(str_list[0])
                _id = _id + 1
                x = float(str_list[1])
                y = float(str_list[2])
                head = float(str_list[3])
                curv = float(str_list[4])
                mode = int(str_list[5])
                velo_mode = int(str_list[6])
                event_mode = int(str_list[7])
                OppositeSideMode = int(str_list[8])
                laneNum = int(str_list[9])
                laneSeq = int(str_list[10])
                laneWidth = float(str_list[11])
                file_write.write("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(_id, x, y, head,curv,mode, velo_mode,event_mode,OppositeSideMode,laneNum,laneSeq,laneWidth))			


if __name__ == "__main__":
    main()
