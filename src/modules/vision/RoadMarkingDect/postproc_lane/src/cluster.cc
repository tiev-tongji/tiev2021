#include <random>
#include "cluster.h"


void cluster::cluster_lanes(
    cv::InputArray lane_mask_, // [in]
    std::vector<Cluster> & clusters // [out]
){
    cv::Mat lane_mask;
    {// preprocess
        const int BIN_THRES = 0;
        const int BLUR_KSIZE = 3;
        const int RE_BIN_THRES = 10;
        const int RE_BIN_MAXVAL = 255;
        if (lane_mask_.channels() == 3){
            cv::cvtColor(lane_mask_, lane_mask, cv::COLOR_BGR2GRAY);
            cv::threshold(lane_mask, lane_mask, BIN_THRES, 255, cv::THRESH_BINARY);//binaryzation
        }else {
            cv::threshold(lane_mask_, lane_mask, BIN_THRES, 255, cv::THRESH_BINARY);
        }
        cv::blur(lane_mask, lane_mask, cv::Size(BLUR_KSIZE, BLUR_KSIZE));
        cv::threshold(lane_mask, lane_mask, RE_BIN_THRES, RE_BIN_MAXVAL, cv::THRESH_BINARY);
    }
    {// cluster
        clusters.clear();
        const float DIFF_Y_RATIO = 0.01f;
        const int CLUSTER_RADIUS = 20;
        auto cluster_new_point = [&](cv::Point2i new_point){
            int best_bin_id = -1; // which existing bin is the best for the new_point?
            float best_diff_xy = 1e6; // what is the distance to the best bin?
            int best_reference_x = -1; // which position is the expectation of a next point for the best bin?
            // Now search for the best(nearest) bin for current new point.
            for(size_t bin_id = 0; bin_id < clusters.size(); bin_id++){
                Cluster & bin = clusters[bin_id]; // Just a short alias
                if (bin.size() == 0) continue; // You cannot match to an empty bin (although there shouldn't be any), try next.
                int reference_x;
                if (bin.size() == 1){
                    reference_x = (*bin.begin()).x;
                }else{
                    reference_x = (*(bin.rbegin()+1)).x;//daoshu scond? why?
                }
                // following is the distance calculation
                float reference_y = (*bin.rbegin()).y;
                float diff_x = (new_point.x - reference_x);
                float diff_y = (new_point.y - reference_y);
                float diff_xy = sqrt(diff_x*diff_x + DIFF_Y_RATIO*diff_y*diff_y) ; // diff_y is less important than diff_x, thus weighted by a ratio (between 0 and 1).
                if (diff_xy < best_diff_xy && diff_xy < CLUSTER_RADIUS){
                    // find the nearest available existing cluster
                    auto pre_point = *clusters[bin_id].rbegin();
                    if (pre_point.y == new_point.y){
                        float diff_pre_x = (pre_point.x - reference_x);
                        float diff_pre_y = (pre_point.y - reference_y);
                        float diff_pre = sqrt(diff_pre_x*diff_pre_x + DIFF_Y_RATIO*diff_pre_y*diff_pre_y) ;
                        if (diff_pre < diff_xy) return;
                        clusters[bin_id].pop_back();
                    }
                    best_diff_xy = diff_xy;
                    best_bin_id = static_cast<int>(bin_id);
                    best_reference_x = reference_x;
                }
            } // done searching for best(nearest) bin.
            if (best_bin_id >= 0) { // found one
                new_point.x = static_cast<int>(std::lround(new_point.x * 0.2 + best_reference_x * 0.8));
                clusters[static_cast<size_t>(best_bin_id)].push_back(new_point);
            }else if(new_point.y > lane_mask.rows*3/4) { // add new cluster, which should not start from far away
                Cluster new_line = {new_point};
                clusters.push_back(new_line);
            }
        }; // end subroutine cluster_new_point()

        // We scan each line from the bottom of the view to the top,
        // and cluster points from each new line to existing or non-existing clusters.
        const int MIN_LINE_WIDTH = 0;
        const int MAX_LINE_WIDTH = 20;
        uchar * mask_data = lane_mask.data + lane_mask.rows * lane_mask.cols; // end of datablock
        for (int row = lane_mask.rows-1; row >=0; --row){
            int foreground = 0;
            mask_data -= lane_mask.cols; // start at the leftmost pixel of the row
            for (int col = 0; col < lane_mask.cols; ++col){
                    if ( ! mask_data[col] ) { // background
                        if ( foreground == 0 ) continue;
                        if (foreground > MIN_LINE_WIDTH && foreground < MAX_LINE_WIDTH) {
                            cluster_new_point(cv::Point2i(col-foreground/2, row));
                        }
                        foreground = 0;
                    }else{
                        foreground ++;
                    }
            } // done with every pixel in one row

        } // end for one row scanning
    }// end of cluster process

    {// remove short lines
        const int MIN_LINE_LENGTH = 160;
        const size_t MIN_LINE_COUNT = 100;
        /* 
		PRINT(clusters.size());
        for(auto& c : clusters){
            PRINT(c.size());
            PRINT((c.front().y - c.back().y));
        }
		*/
        clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [](Cluster& c){
         return (c.size() < MIN_LINE_COUNT) || ((c.front().y - c.back().y) < MIN_LINE_LENGTH);
        }), clusters.end());
    }
}
