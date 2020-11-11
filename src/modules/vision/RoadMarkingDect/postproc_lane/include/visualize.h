#ifndef VISUALIZE_H
#define VISUALIZE_H

#include "common.h"
#include "lane_model.h"
#include "cluster.h"
#include "string.h"

namespace lm = LaneModel;


namespace visualize {

	template <typename T>
	void visualize(const std::vector<cluster::Cluster_<T> > &clusters, // [in]
	    cv::InputOutputArray canvas // [out]
	){
		std::stringstream ss;
		const int CIRCLE_RADIUS = 2;
		const int FILLED = -1;
		const double FONT_SCALE = 1.;
		const int FONT_THICKNESS = 1;
		int idx = 1;
		for (auto cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
			auto color = cluster::random_color();
			ss.str(""); ss << idx++ ;
			cv::putText(canvas, ss.str(), cluster::toIntPoint(cluster->front()) + cv::Point(3, -3),
				    cv::FONT_HERSHEY_PLAIN, FONT_SCALE, color, FONT_THICKNESS);
			for (const cv::Point& point : *cluster) {
			cv::circle(canvas, cluster::toIntPoint(point), CIRCLE_RADIUS, color, FILLED);
			}
		}
	}

	template <typename T>
	void visualize(lm::Road<T>& road, // [in]
        cv::InputOutputArray canvas, // [out]
        cv::Point2i vehicle_position
	){
		std::stringstream ss;
		const int CIRCLE_RADIUS = 2;
		const int FILLED = -1;
		const double FONT_SCALE = 1.7;
		const int FONT_THICKNESS = 2;
		int idx = 0;
		auto lines = road.lines();
		auto lanes = road.lanes();
		std::string linetype_name[4] = {"SW", "DW", "SY", "DY"};
		std::string lanemark_name[9] = {"NONE", "T", "L", "TL", "R", "TR", "INVALID",  "TLR", "U"};
		std::vector<int> linetype_idx, lanemark_idx;
		for (auto& lane : lanes){
			auto color = cluster::random_color();
			cv::Point pos = cluster::toIntPoint(dynamic_cast<lm::Line<float>*>(lane->left())->points().front());
			ss.str(""); 
        	ss << lanemark_name[int(lane->m_marks.back().m_type)];
			cv::putText(canvas, ss.str(), pos + cv::Point(3, -30), cv::FONT_HERSHEY_PLAIN, FONT_SCALE, color, FONT_THICKNESS);

		}

            for (auto line = lines.begin(); line != lines.end(); ++line){
                auto color = cluster::random_color();
            if(idx - 1 == road.get_current_lane_id()){
                ss.str("@");
                cv::putText(canvas, ss.str(), cluster::toIntPoint((*line)->points().front()) + cv::Point(10, -100),
                        cv::FONT_HERSHEY_PLAIN, FONT_SCALE*2, color, FONT_THICKNESS*2);
            }
			ss.str(""); ss << idx++;
			ss << linetype_name[int((*line)->linetype)];
			cv::putText(canvas, ss.str(), cluster::toIntPoint((*line)->points().front()) + cv::Point(3, -10),
				    cv::FONT_HERSHEY_PLAIN, FONT_SCALE, color, FONT_THICKNESS);
			
			for (const cv::Point& point : (*line)->points()) {
				cv::circle(canvas, cluster::toIntPoint(point), CIRCLE_RADIUS, color, FILLED);
			}
			
		}
		

            {
                cv::Point2i top(vehicle_position.x, vehicle_position.y - 80);
                cv::Point2i bot(vehicle_position.x, vehicle_position.y);
                cv::line(canvas, top, bot, cv::Scalar(255, 255, 0), 3);
            }
		
		
	}
	


}

#endif
