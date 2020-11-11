#include <stdio.h>
#include <unistd.h>
#include <zcm/zcm-cpp.hpp>
#include "../include/zcmmsg/MsgRoadMarkingList.hpp"
using std::string;

class Handler
{
    public:
        ~Handler() {}

        void handleMessage(const zcm::ReceiveBuffer* rbuf,
                           const string& chan,
                           const MsgRoadMarkingList *msg)
        {
            printf("num of lanes : _____________%d\n", msg->num);
            printf("current_lane_id: ___________%d\n", msg->current_lane_id);
            int current_lane_id = msg->current_lane_id;
            if(msg->num > 0){
//                printf("current_lane_type: _________%d\n", msg->lanes[current_lane_id].lane_type);
//                printf("current_lane_width: _________%f\n", msg->lanes[current_lane_id].width);
//                printf("current_lane_left_line_type: _%d\n", msg->lanes[current_lane_id].left_line.line_type);
//                printf("current_lane_right_line_type: _%d\n", msg->lanes[current_lane_id].right_line.line_type);
//                printf("stopline_exit: ________________%d\n", msg->stop_line.exist);
//                printf("stopline_points_num: __________%d\n", msg->stop_line.num);
//                printf("stopline_distacne: ____________%f\n", msg->stop_line.distance);
            }
            if(msg->boundary_detected){
                for(int i=0; i<msg->num; i++){
                    if(i==0){
                        if(msg->lanes[0].right_line.boundary_type > 0){
                            printf("boundary type: ___%d      ", msg->lanes[0].right_line.boundary_type);
                            printf("boundary confidence: ___%d\n", msg->lanes[0].right_line.boundary_confidence);
                        }
                    }
                    if(msg->lanes[i].left_line.boundary_type > 0){
                        printf("boundary type: ___%d      ", msg->lanes[i].left_line.boundary_type);
                        printf("boundary confidence: ___%d\n", msg->lanes[i].left_line.boundary_confidence);
                    }
                }
            }
            printf("\n");
        }
};

int main(int argc, char *argv[])
{
    zcm::ZCM zcm{"ipc"};
    //zcm::ZCM zcm {""};
    if (!zcm.good())
        return 1;

    Handler handlerObject;
    zcm.subscribe("LANE_info", &Handler::handleMessage, &handlerObject);
    zcm.run();

    return 0;
}
