#include "virtualscan.h"


using namespace TiEV;
cv::Mat virtualscancheck(GRID_ROW,GRID_COL,CV_8UC1,cv::Scalar(0));

/*
* Author He Xudong
*/
namespace TiEV {
    void generate_virtualscans(structLASERMAP grid, std::vector<Eigen::Vector3f> &virtusalscans)
    {
        grid.rows = GRID_ROW;
        grid.cols = GRID_COL;

        grid.center_row = CAR_CEN_ROW;
        grid.center_col = CAR_CEN_COL;

        cv::Mat scanmat(grid.rows, grid.cols, CV_8UC3,cv::Scalar(0,0,0));
        cv::Point start(grid.center_col, grid.center_row);

        std::vector<cv::Point> endpoints;

        //????push
        //up
        for (int col_index = 0; col_index < grid.cols; col_index += 1)
            endpoints.push_back(cv::Point(col_index, 0));
        //right
        for (int row_index = 0; row_index < grid.rows; row_index += 1)
            endpoints.push_back(cv::Point(grid.cols - 1, row_index));
        //down
        for (int col_index = grid.cols - 1; col_index >= 0; col_index -= 1)
            endpoints.push_back(cv::Point(col_index, grid.rows - 1));
        //left
        for (int row_index = grid.rows - 1; row_index >= 0; row_index -= 1)
            endpoints.push_back(cv::Point(0, row_index));

        std::vector<ScanLine> scanline;
        for (int i = 0; i < endpoints.size(); i++)
        {
            line(scanmat,
                 start,
                 endpoints[i],
                 cv::Scalar(0, 0, 0)    //color??black
            );

            ScanLine sltemp;
            cv::LineIterator it(scanmat, start, endpoints[i], 8);
            std::vector<cv::Point> lptemp;  //??????????
            for (int i = 0; i < it.count; i++, ++it)
                lptemp.push_back(it.pos());
            sltemp.id = i;
            sltemp.state = -1;
            sltemp.endpoint_ori = endpoints[i];
            sltemp.endpoint_access = endpoints[i];
            sltemp.linepoints_ori = lptemp;
            sltemp.linepoints_access = lptemp;
            scanline.push_back(sltemp);
        }

        //????gird????????mat
        for (int i = 0; i < scanmat.rows; i++) {
            for (int j = 0; j < scanmat.cols; j++) {
                if (grid.cells[i][j] == 0x01)
                {
                    scanmat.at<cv::Vec3b>(i, j)[2] = 255;	 //b
                    scanmat.at<cv::Vec3b>(i, j)[1] = 255;	 //g
                    scanmat.at<cv::Vec3b>(i, j)[0] = 255;    //r
                }
            }
        }

        //?????????
        for (size_t lineid = 0; lineid < scanline.size(); lineid++)
        {
            for (size_t pointsid = 0; pointsid < scanline[lineid].linepoints_ori.size(); pointsid++)
            {
                cv::Point middlep(-1, -1), endp(-1, -1);
                middlep.x = scanline[lineid].linepoints_ori[pointsid].x;
                middlep.y = scanline[lineid].linepoints_ori[pointsid].y;

                if (scanmat.at<cv::Vec3b>(middlep.y, middlep.x) == cv::Vec3b(255, 255, 255)
                    /*|| scanmat.at<cv::Vec3b>(middlep.y, middlep.x) == cv::Vec3b(254, 254, 254)*/)
                {
                    scanline[lineid].endpoint_access.x = middlep.x;
                    scanline[lineid].endpoint_access.y = middlep.y;

                    scanline[lineid].linepoints_access.erase(scanline[lineid].linepoints_access.begin() + pointsid,
                                                             scanline[lineid].linepoints_access.end());
                    break;
                }
            }
        }

        //std::cout<<"**********************************once virtualscan*****************"<<std::endl;
        //????????virtualscan
        for (int i = 0; i < scanline.size(); i++)
        {
            if (scanline[i].endpoint_ori == scanline[i].endpoint_access)
            {
                scanline[i].endpoint_access.x = CAR_CEN_COL;
                scanline[i].endpoint_access.y = CAR_CEN_ROW;
                scanline[i].state = 1;
            }
            else
                scanline[i].state = 0;

            line(scanmat,
                 start,
                 scanline[i].endpoint_access,
                 cv::Scalar(143, 246, 255)  // color : light yellow
            );

            float xx = 0, yy = 0;

            xx = ( scanline[i].endpoint_access.x - CAR_CEN_COL) * grid.resolution;
            yy = ( - scanline[i].endpoint_access.y + CAR_CEN_ROW) * grid.resolution;

            //std::cout<<"xx: "<< xx << " , " << yy <<std::endl;

            if(xx == 0 || yy == 0)
                continue;
            //virtusalscans.push_back(Eigen::Vector3f(xx, yy, 0));
            virtusalscans.push_back(Eigen::Vector3f(yy, -xx, 0));
        }

        //std::cout << "----------------------virtusalscans.size():   " << virtusalscans.size()<<std::endl;
//    std::cout<<grid.rows<<","<<grid.cols<<std::endl;

        //?????��?endpoints
//        cv::Mat endptsmat (grid.rows, grid.cols, CV_8UC3,cv::Scalar(0,0,0));
//
//        int endptnum = 0;
//        for (int i = 0; i < scanline.size(); i++)
//        {
//            endptnum++;
//            endptsmat.at<cv::Vec3b>(scanline[i].endpoint_access.y, scanline[i].endpoint_access.x) = cv::Vec3b(0, 0, 255);
//        }

//    cv::namedWindow("endptsmat");
//	cv::imshow("endptsmat", endptsmat);
//    cv::waitKey(1);
//
//	cv::namedWindow("scanmat");
//	cv::imshow("scanmat", scanmat);
//        cv::waitKey(1);
    }
}