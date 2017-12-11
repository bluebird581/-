#ifndef LOCALIZATION_HADMAP_PROCESS_H
#define LOCALIZATION_HADMAP_PROCESS_H

#include "basic_struct.h"
//#include "fastmap.h"
#include <unistd.h>
#include "ransac.hpp"
#include "get_hadmap_data.hpp"
//#include "get_hadmap_data_web.hpp"

//#define OUTPUT_TXT
//#define OUTPUT_TXTQZH

const int im_w = 1792;
const int im_h = 1792;
const int shift_x = im_w >> 3;
const int shift_y = im_h >> 4;
const double scale_x = 16;// 2000000.0;
const double scale_y = 16;// 2000000.0;
const double m_to_pixel = 0.31;//0.31;

const int max_deque = 24; // 3s
const int valid_deque = 8; // 1s

struct POS {
    cv::Point2d loc;
    float angle;
};

struct OLD_RECORD {
    cv::Point2d pt;
    int level;
};

class HadmapProcess {
public:
    //HadmapProcess()
    //{
    //  //ff.open("draw.txt", std::ios::out | std::ios::app);
    //  //ff.setf(std::ios::fixed, ios::floatfield);
    //  //ff.precision(20);

    //  //_fp = fopen("draw.txt", "w");
    //}

    //~HadmapProcess()
    //{
    //  //ff.close();

    //  //fclose(_fp);
    //}
    HadmapProcess() {
#ifdef OUTPUT_TXT
        _fp = fopen("draw.txt", "w");
#endif
#ifdef OUTPUT_TXTQZH
        _fp_qzh = fopen("test0622.txt", "w+");
#endif
        //fastmap_init();
        _kalman_position_y = NULL;
    }
    ~HadmapProcess() {
#ifdef OUTPUT_TXT
        fclose(_fp);
#endif
#ifdef OUTPUT_TXTQZH
        fclose(_fp_qzh);
#endif

        //fastmap_destroy();

        if (!_kalman_position_y) {
            delete _kalman_position_y;
            _kalman_position_y = NULL;
        }
    }
    int do_init(const cv::Rect& roi_out, const std::string db_name) {
        _ransac_fitting.do_init(0.9999, 0.2, 10);
        _roi_out = roi_out;
        //init_from_pb("F:\\10012\\10012_2.pb");
        //zhuxiaohui 6.23
        had::GPS_Coord gps_init;
        gps_init.lon = 115.659167167;
        gps_init.lat = 38.9732831667;
        //      std::string bdpath = "F:\\apollo_data\\apollo_wtf\\ivm_baoding_outer_east_0622_37500.db";
        //      std::string bdpath = "/home/nvidia/liuxueqing/apollo/localization-bosch/data/HAD_MAG_ENGINE_inner_0609/baoding_inner_0620.db";
        //              std::string bdpath = "/home/nvidia/liuxueqing/apollo/localization-bosch/data/apolo_release/ivm_baoding_outer_east_0622_37500.db";
        std::string bdpath = db_name;
        const char* test = db_name.c_str();

        if (access(test, 0) == -1) {
            std::cout << "DB file is not exist!" << std::endl;
            std::cout << "Please check your config file!" << std::endl;
            return -1;
        }

        _pdb = get_haddb();
        bool b_flag = _pdb->initialize(bdpath);
        //std::cout<<b_flag<<std::endl;
        //judge inner or outer xhzhu 0821
        int leng = bdpath.length();
        _judge_outer = bdpath.substr(leng - 13, 5);

        //std::cout << _judge_outer << " _is_outer " << std::endl;
        if (_judge_outer == "outer") {
            _is_outer = true;
        } else {
            _is_outer = false;
        }

        //std::cout<<"~~~~~~~~~~~~~~~~~~~"<<std::endl;
        if (!b_flag) {
            std::cout << "hadmap DB init failed!" << std::endl;
            return -1;
            //std::cout << "\n DB init success! \n" << std::endl;
        }

        _pdb->init_start_position(gps_init);
        srand((unsigned)time(NULL));
        return 0;
    }

public:
    SYSTEM_STATUS do_query_hadmap(
        const cv::Vec3d& gps,
        DYNAMIC_PARAMS& para,
        RETINFO& ret_info);

    SYSTEM_STATUS do_delta_y_estimation(
        RETINFO& ret_info);
    SYSTEM_STATUS do_delta_x_estimation(
        RETINFO& ret_info);

private:
    int require_hadmap_data_from_engine(const had::GPS_Coord& gps,
                                        std::vector <LANE_MARKING_HAD>& lanes_had,
                                        std::vector <ARROWS_HAD>& arrows_had);
    int require_hadmap_data(const cv::Vec3d& gps, RETINFO& ret_info);
    int rotate_for_refinement(const cv::Point2d& center_pt,
                              RETINFO& ret_info);

private:
    had::IHadDB* _pdb; //zhuxiaohui 6.23

    std::deque <cv::Point2d> _pts_outer_filter;

    cv::Rect _roi_out;

    int _slam_lasting;

    HadmapData _hadmap_query;
    //HadmapDataWeb _hadmap_query_web;
    RansacFitting _ransac_fitting;
    Kalman2* _kalman_position_y;

    //IPMTransformer _ipm_transformer;

    std::deque<POS> _traces;

    std::vector<LANE_COMBO> _lanes_combo;
    std::vector<ARROW> _arrows;

    float _angle;

    float _angle_basic;

    bool _is_outer; //0821 for add diff pianyi xhzhu
    std::string _judge_outer;

    //std::ofstream ff;
#ifdef OUTPUT_TXT
    FILE* _fp;
#endif
#ifdef OUTPUT_TXTQZH
    FILE* _fp_qzh;
#endif

    Timer _timer;

    cv::Point2d _initial_center_pt;
    cv::Point2d _refined_y_center_pt;
    std::vector<cv::Point2d> _old_gt;
    std::deque<cv::Point2d> _old_estimations;
    double _old_shift_y;

    std::deque<OLD_RECORD> _pt_estimations;

    std::vector<had::Lane*> _p_lane_vec;
    std::vector<had::LaneTopology*> _p_lane_topo_vec;
    std::vector<had::Object*> _p_object_vec;
    std::vector<had::Link*> _p_link_vec;
    std::vector<had::LinkTopology*> _p_link_topology_vec;
};

bool sort_by_x_asc(const LANE& l1, const LANE& l2) {
    return l1.end_pt_ipm.x < l2.end_pt_ipm.x;
}

/*
 * @para hskdf
 *
 */
cv::Point2d lonlat_to_mercator_zhu(const cv::Point2d& lonlat) {
    cv::Point2d mercator;
    mercator.x = lonlat.x * 20037508.34 / 180;
    mercator.y = log(tan((90 + lonlat.y) * CV_PI / 360)) / (CV_PI / 180);
    mercator.y = mercator.y * 20037508.34 / 180;
    return mercator;
}

double cal_distance(cv::Point2d& initial_center_pt, cv::Point2d& start, cv::Point2d& end) {
    // L: 20170721
    // caculate dis between initial_pt, start point and end point
    // if project of initial_pt is not between start and end, then the dis is equal to min distance of start and end,
    // else the dis is distance from initial_point to the lines
    double dis;
    start.x -= initial_center_pt.x;
    start.y -= initial_center_pt.y;
    end.x -= initial_center_pt.x;
    end.y -= initial_center_pt.y;
    double k = ((start.x - end.x) * start.x + (start.y - end.y) * start.y) / (pow(start.x,
               2) + pow(start.y, 2));

    if (k < 0 || k > 1) {
        dis = std::min(pow(pow(start.x, 2) + pow(start.y, 2), 0.5), pow(pow(end.x, 2) 
        + pow(end.y, 2), 0.5));
    } else {
        dis = pow(pow((1 - k) * start.x + k * end.x, 2) + pow((1 - k) * start.y + k 
        * end.y, 2), 0.5);
    }

    return dis;
}

double cal_heading(cv::Point2d& start, cv::Point2d& end) {
    double heading = 0.0;

    if (start.y == end.y) {
        heading = CV_PI / 2;
    } else {
        heading = atan((end.x - start.x) / (end.y - start.y));
    }

    double y0 = 0.0;
    double y1 = 0.0;
    y0 = sin(heading) * start.x + cos(heading) * start.y;
    y1 = sin(heading) * end.x + cos(heading) * end.y;

    if (y0 > y1) {
        heading += CV_PI;
    }

    return heading * 180 / CV_PI;
}

const double image_cols = 1000.0;
const double image_rows = 500.0;
const double merter_width = 100.0; // m
const double merter_height = 50.0;
const double dw = image_cols / merter_width;
const double dh = image_rows / merter_height;

cv::Point2d mercator_to_image(const cv::Point2d& mercator_point, const cv::Point2d& center_pt) {
    cv::Point2d point = mercator_point - center_pt;
    // zoom in image scale
    point.x *= dw;
    point.y *= dh;
    point.x += (image_cols * 0.5);
    point.y += (image_rows * 0.5);
    point.y = image_rows - point.y;
    return point;
}
void plot_lanes_arrows_fangli(had::GPS_Coord& coord, std::vector <LANE_MARKING_HAD>& lanes_had,
                              std::vector <ARROWS_HAD>& arrows_had, std::string pic_name) {
    cv::Point2d p_init(coord.lon, coord.lat);
    p_init = lonlat_to_mercator_zhu(p_init);
    cv::Mat img(image_rows, image_cols, CV_8UC3, cv::Scalar(48, 48, 48));
    //     cv::namedWindow("IMGhad");
    std::vector<std::vector<cv::Point> > contours;
    //judge start point in pic in pixel level
    cv::Point2d start;
    start.x = 0;//colums;
    start.y = 0;//rows;    //逆时针直接指定即可，顺时针则指定为（0,0）
    //do transform for each point
    std::vector <int> type;

    for (int i = 0; i < lanes_had.size(); ++i) {//marking_list.size()
        //std::cout << i << "   " << j << std::endl;
        LANE_MARKING_HAD temp = lanes_had[i];
        std::vector<cv::Point2d> points = temp.points_line;
        //std::cout << "points   " << points.size() << std::endl;
        type.push_back(temp.lane_marking_type);
        //print lane marks
        std::vector<cv::Point> contour;

        for (int m = 0; m < points.size(); ++m) {
            //             std::cout << setprecision(11) << points[m].x <<"   "<< points[m].y <<"   "<< points[m].z <<"   "<<std::endl;
            cv::Point2d p1(points[m].x, points[m].y);
            p1 = mercator_to_image(p1, p_init);
            //             printf("(%.2lf, %.2lf) ", p1.x, p1.y);
            contour.push_back(p1);
        }

        //         printf("\n");
        contours.push_back(contour);
    }

    // 画原始GPS点
    cv::circle(img, cv::Point2d(image_cols * 0.5, image_rows * 0.5), 5, CV_RGB(255, 0, 0), -1);

    //画车道线
    //std::ofstream fout;
    //fout.open("sb.txt", std::ios::app);
    for (int i = 0; i < contours.size(); i++) { //contours.size()
        for (int j = 0; j < contours[i].size() - 1; j++) { //
            //printf("(%d, %d) ", contours[i][j].x, contours[i][j].y);
            //fout << contours[i][j].y << " ";
            if (type[i] == 0 || type[i] == 2) {
                cv::line(img, contours[i][j], contours[i][j + 1], cv::Scalar(255, 255, 255));
                cv::circle(img, contours[i][j], 3, CV_RGB(255, 255, 255));
            } else if (type[i] == 1 || type[i] == 3) {
                cv::line(img, contours[i][j], contours[i][j + 1], cv::Scalar(255, 255, 0));
                cv::circle(img, contours[i][j], 3, CV_RGB(255, 255, 0));
                //std::cout << contours[i][j].x << "  " << contours[i][j].y
                //  <<  ")  " << contours[i][j+1].x << "  " << contours[i][j+1].y << std::endl;
            } else {
                //cv::line(img,contours[i][j],contours[i][j+1],cv::Scalar(255,0,0),0.4);  //其他类型先不画
            }

            //cv::circle(img, contours[i][j], 0.4, CV_RGB(255, 255, 0), -1);//
        }

        //std::cout << std::endl;
        //fout << std::endl;
    }

    //fout.close();
    //再画出箭头
    std::vector <std::vector <cv::Point> > contours_arrow;

    for (int i = 0; i < arrows_had.size(); ++i) {
        std::vector <cv::Point> contour;

        for (int j = 0; j < arrows_had[i].corner_points.size(); ++j) {
            cv::Point2d p1(arrows_had[i].corner_points[j].x, arrows_had[i].corner_points[j].y);
            p1 = mercator_to_image(p1, p_init);
            contour.push_back(p1);
        }

        contours_arrow.push_back(contour);
    }

    cv::drawContours(img, contours_arrow, -1, cv::Scalar(255, 255, 0), 1);
    //final show
#ifdef localization_display
    cv::imshow("had_img", img);
#endif
    //cv::waitKey(0);
}

void plot_lanes_arrows_new(had::GPS_Coord& coord, std::vector <LANE_MARKING_HAD>& lanes_had,
                           std::vector <ARROWS_HAD>& arrows_had, std::string pic_name) {
    std::cout << pic_name << std::endl;
    std::cout << "ppppppp" << lanes_had.size() << std::endl;
    double scale_new = 100; // m
    //标准对照点
    //cv::Point2d p_init(115.574396565, 38.8505524233);
    //cv::Point2d p_init(0, 0);
    cv::Point2d p_init(coord.lon, coord.lat);
    p_init = lonlat_to_mercator_zhu(p_init);
    //coord.lon = 0; coord.lat = 0;
    int rows = 500;
    int colums = 1000;
    cv::Mat img(rows, colums, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::namedWindow("IMGhad");
    std::vector<std::vector<cv::Point> > contours;
    //judge start point in pic in pixel level
    cv::Point2d start;
    start.x = 0;//colums;
    start.y = 0;//rows;    //逆时针直接指定即可，顺时针则指定为（0,0）
    //do transform for each point
    std::vector <int> type;

    for (int i = 0; i < lanes_had.size(); ++i) {//marking_list.size()
        //std::cout << i << "   " << j << std::endl;
        LANE_MARKING_HAD temp = lanes_had[i];
        std::vector<cv::Point2d> points = temp.points_line;
        //std::cout << "points   " << points.size() << std::endl;
        type.push_back(temp.lane_marking_type);
        //print lane marks
        std::vector<cv::Point> contour;

        for (int m = 0; m < points.size(); ++m) {
            //std::cout << setprecision(11) << points[m].x <<"   "<< points[m].y <<"   "<< points[m].z <<"   "<<std::endl;
            cv::Point2d p1(points[m].x, points[m].y);
            p1.x = p1.x - p_init.x;
            p1.y = p1.y - p_init.y;
            p1.x = p1.x / scale_new * colums + start.x;
            p1.y = p1.y / scale_new * rows + start.y; //colums / rows
            //              p1.x = - p1.x;
            p1.y = -p1.y;
            //std::cout << p1.x << " point " << p1.y << std::endl;
            contour.push_back(p1);
        }

        //reverse
        std::vector<cv::Point> contour_final;

        for (int i = 0; i < contour.size(); ++i) {
            contour_final.push_back(contour[i]);
        }

        contours.push_back(contour_final);
    }

    //std::cout << "contours.size = " << contours.size() << std::endl;
    //画出汽车位置  cpt
    cv::Point2d p1_1(coord.lon, coord.lat);
    p1_1 = lonlat_to_mercator_zhu(p1_1);
    p1_1.x = p1_1.x - p_init.x;
    p1_1.y = p1_1.y - p_init.y;
    cv::Point p11;
    p11.x = p1_1.x / scale_new * colums + start.x;
    p11.y = p1_1.y / scale_new * rows + start.y;
    //画得太小了，抠出来局部放大
    //通过当前点去筛选出，小范围车道线
    //      p11.x = - p11.x;
    p11.y = -p11.y;
    int move_x = 500 - p11.x;
    int move_y = 250 - p11.y;

    for (int i = 0; i < contours.size(); i++) {
        for (int j = 0; j < contours[i].size(); j++) { //出大错了，就一点没减，contours[i].size()-1
            contours[i][j].x += move_x;
            contours[i][j].y += move_y; //
            std::cout << contours[i][j].x << " point: " << contours[i][j].y << std::endl;
        }
    }

    p11.x = 500;
    p11.y = 250;

    //画车道线
    //std::ofstream fout;
    //fout.open("sb.txt", std::ios::app);
    for (int i = 0; i < contours.size(); i++) { //contours.size()
        for (int j = 0; j < contours[i].size(); j++) {
            //printf("(%d, %d) ", contours[i][j].x, contours[i][j].y);
            //fout << contours[i][j].x << " ";
            //cv::circle(img, contours[i][j], 2, CV_RGB(255, 255, 0), -1);
        }

        //printf("\n");
        //fout << std::endl;
        for (int j = 0; j < contours[i].size(); j++) {
            //fout << contours[i][j].y << " ";
        }

        for (int j = 0; j < contours[i].size() - 1; j++) { //
            //printf("(%d, %d) ", contours[i][j].x, contours[i][j].y);
            //fout << contours[i][j].y << " ";
            if (type[i] == 0 || type[i] == 2) {
                cv::line(img, contours[i][j], contours[i][j + 1], cv::Scalar(255, 255, 255), 0.4);
            } else if (type[i] == 1 || type[i] == 3) {
                cv::line(img, contours[i][j], contours[i][j + 1], cv::Scalar(255, 255, 0), 0.4);
                //std::cout << contours[i][j].x << "  " << contours[i][j].y
                //  <<  ")  " << contours[i][j+1].x << "  " << contours[i][j+1].y << std::endl;
            } else {
                //cv::line(img,contours[i][j],contours[i][j+1],cv::Scalar(255,0,0),0.4);  //其他类型先不画
            }

            //cv::circle(img, contours[i][j], 0.4, CV_RGB(255, 255, 0), -1);//
        }

        //std::cout << std::endl;
        //fout << std::endl;
    }

    //fout.close();
    //画两个点
    //  cv::circle(img, p11, 4, CV_RGB(255, 255, 0), -1); // cpt
    //cv::circle(img, p_ub1, 3, CV_RGB(0, 0, 255), -1); // ublox
    //draw true pic in the subwindow
    //std::cout << pic_num << std::endl;
    //再画出箭头
    std::vector <std::vector <cv::Point> > contours_arrow;

    for (int i = 0; i < arrows_had.size(); ++i) {
        std::vector <cv::Point> contour;

        for (int j = 0; j < arrows_had[i].corner_points.size(); ++j) {
            cv::Point2d p1(arrows_had[i].corner_points[j].x, arrows_had[i].corner_points[j].y);
            p1.x = p1.x - p_init.x;
            p1.y = p1.y - p_init.y;
            p1.x = p1.x / scale_new * colums + start.x;
            p1.y = p1.y / scale_new * rows + start.y; //colums / rows
            //p1.x = - p1.x;
            p1.y = -p1.y;
            contour.push_back(p1);
        }

        contours_arrow.push_back(contour);
    }

    for (int i = 0; i < contours_arrow.size(); i++) {
        for (int j = 0; j < contours_arrow[i].size(); j++) {
            contours_arrow[i][j].x += move_x;
            contours_arrow[i][j].y += move_y;
            //std::cout << contours_arrow[i][j].x << "   " << contours_arrow[i][j].y << std::endl;
        }
    }

    cv::drawContours(img, contours_arrow, -1, cv::Scalar(255, 255, 0), 1);
    //final show
    cv::imshow("IMGhad", img);
    //     cv::waitKey(600);
}

int HadmapProcess::require_hadmap_data_from_engine(const had::GPS_Coord& gps,
        std::vector <LANE_MARKING_HAD>& lanes_had,
        std::vector <ARROWS_HAD>& arrows_had) {
    std::vector <LANE_MARKING_HAD>& lanes_combo = lanes_had;
    std::vector <ARROWS_HAD>& arrows = arrows_had;
    // 1. the basis setup
    lanes_combo.clear();
    arrows.clear();
    double radium = 0; //(rand() % 9 * 10 + 10);//0
    double theta = 0;// *CV_PI / 180; // (rand() % 45 * 8) * CV_PI / 180;//0
    //通过match找到车道ID，section ID，然后再次取出整个section
    int lane_id = 0;
    int link_id = 0; //section
    std::vector<had::Lane*> lanes_section;
    _pdb->matching(gps, lane_id, link_id);
    _pdb->get_around_map(gps, 10, &_p_object_vec, &_p_lane_vec, &_p_lane_topo_vec, &_p_link_vec,
                         &_p_link_topology_vec);
    std::set<int> id_set;

    if (0 != link_id) {
        id_set.insert(link_id);
        _pdb->get_lane_info(1, id_set, &lanes_section);
        std::cout << "match  link_id: " << link_id << std::endl;
        std::cout << gps.lon << "\t" << gps.lat << std::endl;
        std::cout << "lanes_section size: " << lanes_section.size() << std::endl;
    } else {
        if (_p_link_vec.size() == 1) {
            id_set.insert(_p_link_vec[0]->id);
            _pdb->get_lane_info(1, id_set, &lanes_section);
            std::cout << "around map: " << _p_link_vec[0]->id << std::endl;
        } else {
            std::cout << "matching_failed!, try " << _p_link_vec.size() << " links" << std::endl;
            double g_min_dis = INT_MAX;
            //            std::vector <had::Link*>::iterator it;
            int size = _p_link_vec.size();

            for (int i = 0; i < size; i++) {
                // L: 根据历史 heading 角过滤 section
                std::vector<had::Lane*> lanes_section_tmp;
                std::set<int> id_tmp;
                id_tmp.insert(_p_link_vec[i]->id);
                double min_dis = INT_MAX;
                _pdb->get_lane_info(1, id_tmp, &lanes_section_tmp);

                // L: 外层循环, section 的所有车道线
                for (int i = 0; i < lanes_section_tmp.size(); ++i) {
                    double min_dis_inner = INT_MAX;
                    int point_id = 0;
                    cv::Point2d p1;
                    cv::Point2d p2;

                    for (int j = 0; j < lanes_section_tmp[i]->geometry.size() - 1; j++) {
                        cv::Point2d p2d_before(lanes_section_tmp[i]->geometry[j].x, 
                                               lanes_section_tmp[i]->geometry[j].y);
                        cv::Point2d p2d_after(lanes_section_tmp[i]->geometry[j + 1].x,
                                              lanes_section_tmp[i]->geometry[j + 1].y);
                        p2d_before = lonlat_to_mercator_zhu(p2d_before);
                        p2d_after = lonlat_to_mercator_zhu(p2d_after);
                        double distance = pow(pow(p2d_before.x - _initial_center_pt.x,
                                                  2) + pow(p2d_before.y - 
                                                  _initial_center_pt.y, 2), 0.5)
                                          + pow(pow(p2d_after.x - _initial_center_pt.x, 2) 
                                          + pow(p2d_after.y - _initial_center_pt.y, 2), 0.5);

                        if (distance < min_dis_inner) {
                            min_dis_inner = distance;
                            point_id = j;
                            p1 = p2d_before;
                            p2 = p2d_after;
                        }
                    }

                    //cv::Point2d p1(tmp.geometry[point_id].x, tmp.geometry[point_id].y);
                    //cv::Point2d p2(tmp.geometry[point_id + 1].x, tmp.geometry[point_id + 1].y);
                    double heading_tmp = cal_heading(p1, p2);
                    double min_dis_tmp = cal_distance(_initial_center_pt, p1, p2);

                    if (min_dis_tmp < min_dis) {
                        min_dis = min_dis_tmp;
                    }
                }

                if (min_dis < g_min_dis) {
                    g_min_dis = min_dis;
                    lanes_section = lanes_section_tmp;
                }
            }
        }
    }

    std::cout << "lane_section.size =========== " << lanes_section.size() << std::endl;
    //从接口数据挑选出从左到右的车道线
    bool flag = true;
    std::vector <had::LaneMarking*> lane_markings_section;

    for (int i = lanes_section.size(); i > 0; --i) {
        int j = lanes_section.size() - 1;

        while (j >= 0) {
            if (lanes_section[j]->sequence == i) {
                if (flag) {
                    //压入左边
                    for (int k = 0; k < lanes_section[j]->marking_list.size(); ++k) {
                        //side 弄反就多一根线
                        if (lanes_section[j]->marking_list[k]->side == 0 &&
                                (lanes_section[j]->marking_list[k]->lane_marking_type == 1 ||
                                 lanes_section[j]->marking_list[k]->lane_marking_type == 3)) {
                            lane_markings_section.push_back(lanes_section[j]->marking_list[k]);
                            //for apollo *** std::cout << "aaaa   " << lanes_section[j]->marking_list[k]->lane_marking_type << std::endl;
                        }
                    }

                    //压入右边
                    for (int k = 0; k < lanes_section[j]->marking_list.size(); ++k) {
                        if (lanes_section[j]->marking_list[k]->side == 1 &&
                                (lanes_section[j]->marking_list[k]->lane_marking_type == 1 ||
                                 lanes_section[j]->marking_list[k]->lane_marking_type == 3)) {
                            lane_markings_section.push_back(lanes_section[j]->marking_list[k]);
                            //for apollo *** std::cout << "bbbb  " << lanes_section[j]->marking_list[k]->lane_marking_type << std::endl;
                        }
                    }

                    flag = false; //第一次才压入两边
                } else {
                    //压入右边
                    for (int k = 0; k < lanes_section[j]->marking_list.size(); ++k) {
                        if (lanes_section[j]->marking_list[k]->side == 1 &&
                                (lanes_section[j]->marking_list[k]->lane_marking_type == 1 ||
                                 lanes_section[j]->marking_list[k]->lane_marking_type == 3)) {
                            lane_markings_section.push_back(lanes_section[j]->marking_list[k]);
                            //for apollo *** std::cout << "cccc   " << lanes_section[j]->marking_list[k]->lane_marking_type << std::endl;
                        }
                    }
                }
            }

            --j;
        }
    }

    //检验，汇入口可能存在边缘车道划到其他section的问题
    //std::vector <std::vector<had::LaneMarking*> > vec_lane_markings_section;
    //vec_lane_markings_section.push_back(lane_markings_section);
    //plot(vec_lane_markings_section,coord,coord,pic_nums[shunxu]);
    had::GPS_Coord gps1 = gps;

    //plot_lanes_arrows_new(gps1, lanes_had, arrows_had, "");
    //plot(vec_lane_markings_section,coord,coord,pic_nums[shunxu],-1);
    //std::cout << " 111111" << std::endl;
    for (int i = 0; i < lane_markings_section.size(); ++i) {
        LANE_MARKING_HAD lane_mark;
        had::LaneMarking tmp = *(lane_markings_section[i]);
        lane_mark.id = i;

        //type
        //LANEMARK_TYPE_WHITE_DASHED = 0,  /*!< 白色虚线 */
        //LANEMARK_TYPE_WHITE_SOLID = 1,   /*!< 白色实线 */
        //LANEMARK_TYPE_YELLOW_DASHED = 2, /*!< 黄色虚线 */
        //LANEMARK_TYPE_YELLOE_SOLID = 3   /*!< 黄色实线 */
        //white 2 yellow 1 ;dashed 1, single solid 3;solid还有很多种
        if (tmp.color == 2 && tmp.lane_marking_type == 1) {
            lane_mark.lane_marking_type = LANEMARK_TYPE_WHITE_DASHED;
        }

        if (tmp.color == 2 && tmp.lane_marking_type == 3) {
            lane_mark.lane_marking_type = LANEMARK_TYPE_WHITE_SOLID;
        }

        if (tmp.color == 1 && tmp.lane_marking_type == 1) {
            lane_mark.lane_marking_type = LANEMARK_TYPE_YELLOW_DASHED;
        }

        if (tmp.color == 1 && tmp.lane_marking_type == 3) {
            lane_mark.lane_marking_type = LANEMARK_TYPE_YELLOE_SOLID;
        }

        //others
        for (int j = 0; j < tmp.geometry.size(); ++j) {
            //add mercator
            cv::Point2d p2d(tmp.geometry[j].x, tmp.geometry[j].y);
            //std::cout << std::setprecision(10) << p2d.x << " line " << p2d.y << std::endl;
            p2d = lonlat_to_mercator_zhu(p2d);
            //std::cout << p2d.x << " mercator line " << p2d.y << std::endl;
            lane_mark.points_line.push_back(p2d);
            cv::Point2d spg(gps.lon, gps.lat);
            spg = lonlat_to_mercator_zhu(spg);
        }

        //起点和终点要确认
        lane_mark.start_pt = lane_mark.points_line[0];
        lane_mark.end_pt = lane_mark.points_line[tmp.geometry.size() - 1];
        lanes_had.push_back(lane_mark);
    }

    if (_p_object_vec.size() > 0 && _p_object_vec[0]->geometry.size() > 1) {
        //std::cout << "1" << std::endl;
        for (int k = 0; k < _p_object_vec.size(); ++k) {
            //std::cout << "2" << std::endl;
            if (_p_object_vec[k]->subtype == 1) {
                //std::cout << _p_object_vec[k]->subtype << "size:  " << _p_object_vec[k]->geometry.size() << std::endl;
                ARROWS_HAD arrow_had;

                for (int m = 0; m < _p_object_vec[k]->geometry.size(); ++m) {
                    //add mercator
                    cv::Point2d p2d(_p_object_vec[k]->geometry[m].x, 
                                    _p_object_vec[k]->geometry[m].y);
                    p2d = lonlat_to_mercator_zhu(p2d);
                    arrow_had.corner_points.push_back(p2d);
                }

                arrows_had.push_back(arrow_had);
            }
        }
    }

    //std::cout << " 555555" << std::endl;
    //_pdb->free_lane_info(&_p_lane_vec);
    //_pdb->free_obj_info(&_p_object_vec);
    //_pdb->free_lane_topo_info(&_p_lane_topo_vec);
    plot_lanes_arrows_fangli(gps1, lanes_had, arrows_had, "");

    //     plot_lanes_arrows_new(gps1, lanes_had, arrows_had, "");
    //Sleep(0);
    //6.17  下面这些有啥用
    //ret_info.pt_gps_draw = _initial_center_pt;
    //ret_info.pt_gps = _initial_center_pt;
    //ret_info.pt_ground_truth_draw = _hadmap_query.lonlat_to_mercator(cv::Point2d(gps.val[0], gps.val[1]));
    //ret_info.pt_ground_truth = ret_info.pt_ground_truth_draw;
    if (!lanes_combo.size()) {
        return -1;
    }

    return 0;
}

int HadmapProcess::require_hadmap_data(const cv::Vec3d& gps, RETINFO& ret_info) {
    //for apollo *** std::cout << "    [+] Begin Requiring Hadmap Data" << std::endl;
    //_timer.do_start_timer();
    std::vector<LANE_COMBO>& lanes_combo = ret_info.lanes_combo;
    std::vector<ARROW>& arrows = ret_info.arrows;
    // 1. the basis setup
    lanes_combo.clear();
    arrows.clear();
    double radium = 0;// (rand() % 9 * 10 + 10);
    double theta = 0;// (rand() % 45 * 8) * CV_PI / 180;
    // 4. rotate the lane lines
    _initial_center_pt = cv::Point2d(gps.val[0] + (radium * cos(theta)) * 0.000001,
                                     gps.val[1] + (radium * sin(theta)) * 0.000001);
    std::string ret_string;
    //fastmap_query(_initial_center_pt.x, _initial_center_pt.y, ret_info.lanes_combo, arrows);
    // zhuxiaohui 6.23
    had::GPS_Coord gps1;
    //gps1.lon = 115.544366167;
    //gps1.lat = 38.7977318333;
    std::vector <LANE_MARKING_HAD> lanes_had;
    std::vector <ARROWS_HAD> arrows_had;
    had::GPS_Coord zxhgps;
    zxhgps.lat = gps.val[1] + (radium * sin(theta)) * 0.000001;
    zxhgps.lon = gps.val[0] + (radium * cos(theta)) * 0.000001;
    require_hadmap_data_from_engine(zxhgps, lanes_had, arrows_had);
    std::cout << "do good job ! Offers " << lanes_had.size() << " lanes and  " 
            << arrows_had.size() << "  arrows !"  << std::endl;
    // quanyun 6.23
    LANE_MARKING_HAD new_lane;
    ARROWS_HAD new_arrow;
    LANE_COMBO old_lane_combo;
    ARROW old_arrow;
    WEIGHTEDPOINT t_oldpt;

    for (int i = 0; i < lanes_had.size(); i++) {
        new_lane = lanes_had[i];
        LANE old_lane;

        for (int j = 0; j < new_lane.points_line.size(); ++j) {
            t_oldpt.pt_d = new_lane.points_line[j];
            old_lane.pts_ipm.push_back(t_oldpt);
        }

        if (new_lane.lane_marking_type == LANEMARK_TYPE_WHITE_SOLID
                || new_lane.lane_marking_type == LANEMARK_TYPE_YELLOE_SOLID) {
            old_lane.type = 1;
        } else {
            old_lane.type = 0;
        }

        old_lane_combo.lanes.push_back(old_lane);
    }

    LANE_COMBO hebei_lane_combo;
    cv::Point2d temp = _hadmap_query.lonlat_to_mercator(_initial_center_pt);
    int index = -1;
    int index_by_x = -1;
    int time_by_x = 0;
    int index_by_y = -1;
    int time_by_y = 0;

    //_timer.do_start_timer();
    for (int i = 0; i < old_lane_combo.lanes.size(); i++) {
        LANE xiangdui_lane;
        std::vector<double> xiangdui_dist;
        int timaa = 0;
        double min_dist = 10000000;

        for (int j = 0; j < old_lane_combo.lanes[i].pts_ipm.size(); j++) {
            t_oldpt.pt_d.x = old_lane_combo.lanes[i].pts_ipm[j].pt_d.x - temp.x;
            t_oldpt.pt_d.y = old_lane_combo.lanes[i].pts_ipm[j].pt_d.y - temp.y;
            //std::cout << t_oldpt.pt_d.x << "  zhunahuan: " << t_oldpt.pt_d.y << std::endl;

            //delete with under index_right/index_left calc 0826
            if (j != 0 && t_oldpt.pt_d.y * xiangdui_lane.pts_ipm[j - 1].pt_d.y < 0) {
                index_by_y = j - 1;
                ++time_by_y;
            }

            if (j != 0 && t_oldpt.pt_d.x * xiangdui_lane.pts_ipm[j - 1].pt_d.x < 0) {
                index_by_x = j - 1;
                ++time_by_x;
            }

            if (j == 0) {
                double temp_dist = 0;
                xiangdui_dist.push_back(temp_dist);
            } else {
                double temp_dist = sqrt((t_oldpt.pt_d.x - xiangdui_lane.pts_ipm[j - 1].pt_d.x) *
                                        (t_oldpt.pt_d.x - xiangdui_lane.pts_ipm[j - 1].pt_d.x) +
                                        (t_oldpt.pt_d.y - xiangdui_lane.pts_ipm[j - 1].pt_d.y) * 
                                        (t_oldpt.pt_d.y - xiangdui_lane.pts_ipm[j - 1].pt_d.y));
                xiangdui_dist.push_back(temp_dist);
            }

            xiangdui_lane.pts_ipm.push_back(t_oldpt);
        }

        xiangdui_lane.type = old_lane_combo.lanes[i].type;

        if (time_by_y == 1) {
            index = index_by_y;
        } else {
            index = index_by_x;
        }

        //find 80meter
        int index_left = index;
        int index_right = index + 1;
        double total_dist = 0;

        while (index_left >= 0 || index_right < xiangdui_dist.size()) {
            if (index_left >= 0) {
                total_dist += xiangdui_dist[index_left];
                index_left--;
            }

            if (index_right < xiangdui_dist.size()) {
                total_dist += xiangdui_dist[index_right];
                index_right++;
            }

            if (total_dist > 160) {
                break;
            }
        }

        if (index_left < 0) {
            index_left = 0;
        }

        if (index_right > xiangdui_dist.size() - 1) {
            index_right = xiangdui_dist.size() - 1;
        }

        LANE lane80;

        for (int k = index_left; k < index_right && k < xiangdui_dist.size() - 1; k++) {
            double dx = xiangdui_lane.pts_ipm[k + 1].pt_d.x - xiangdui_lane.pts_ipm[k].pt_d.x;
            double dy = xiangdui_lane.pts_ipm[k + 1].pt_d.y - xiangdui_lane.pts_ipm[k].pt_d.y;
            double xylength = sqrt(dx * dx + dy * dy);
            dx /= xylength;
            dy /= xylength;
            double xs = xiangdui_lane.pts_ipm[k].pt_d.x;
            double ys = xiangdui_lane.pts_ipm[k].pt_d.y;
            double xe = xiangdui_lane.pts_ipm[k + 1].pt_d.x;
            double ye = xiangdui_lane.pts_ipm[k + 1].pt_d.y;
            WEIGHTEDPOINT temp_wpt;
            temp_wpt.pt_d.x = xs + temp.x;
            temp_wpt.pt_d.y = ys + temp.y;
            int step = 1;

            while (step < xiangdui_dist[k + 1]) {
                step++;
                lane80.pts_ipm.push_back(temp_wpt);
                xs += dx;
                ys += dy;
                temp_wpt.pt_d.x = xs + temp.x;
                temp_wpt.pt_d.y = ys + temp.y;
            }

            temp_wpt.pt_d.x = xe + temp.x;
            temp_wpt.pt_d.y = ye + temp.y;
            lane80.pts_ipm.push_back(temp_wpt);
        }

        lane80.type = xiangdui_lane.type;
        hebei_lane_combo.lanes.push_back(lane80);
        //hebei_lane_combo.lanes.push_back(xiangdui_lane);
    }

    hebei_lane_combo.center_pt = old_lane_combo.center_pt;
    //_timer.do_end_timer("chazhi ####");
    ret_info.lanes_combo.push_back(hebei_lane_combo);

    for (int i = 0; i < arrows_had.size(); i++) {
        new_arrow = arrows_had[i];
        old_arrow.pts_ipm = new_arrow.corner_points;
        //old_arrow.id = 0;
        ret_info.arrows.push_back(old_arrow);
    }

    _initial_center_pt = _hadmap_query.lonlat_to_mercator(_initial_center_pt);
    ret_info.pt_gps_draw = _initial_center_pt;
    ret_info.pt_gps = _initial_center_pt;
    ret_info.pt_ground_truth_draw = _hadmap_query.lonlat_to_mercator(cv::Point2d(gps.val[0],
                                    gps.val[1]));
    ret_info.pt_ground_truth = ret_info.pt_ground_truth_draw;
#ifdef OUTPUT_TXT
    _fprintf(_fp, "%.20f,%.20f,%.20f,%.20f\n", ret_info.pt_ground_truth_draw.x,
            ret_info.pt_ground_truth_draw.y, ret_info.pt_gps_draw.x,
            ret_info.pt_gps_draw.y);
#endif
#ifdef OUTPUT_TXT

    for (int i = 0; i < lanes_combo.size(); ++i) {
        if (lanes_combo[i].lanes.size() >= 1) {
            lanes_combo[i].lanes[0].type = 1;
            lanes_combo[i].lanes[lanes_combo[i].lanes.size() - 1].type = 1;

            if (lanes_combo[i].lanes.size() == 5) {
                lanes_combo[i].lanes[3].type = 2;
            } else if (lanes_combo[i].lanes.size() == 6) {
                lanes_combo[i].lanes[4].type = 2;
                lanes_combo[i].lanes[1].type = 0;
            }
        }

        for (int j = 0; j < lanes_combo[i].lanes.size(); ++j) {
            _fprintf(_fp, "%d,", lanes_combo[i].lanes[j].type);

            for (int k = 0; k < lanes_combo[i].lanes[j].pts_ipm.size(); ++k) {
                //ff << arrows[i].pts_ipm[j].x << " " << arrows[i].pts_ipm[j].y << " ";
                _fprintf(_fp, "%.20f,%.20f,", lanes_combo[i].lanes[j].pts_ipm[k].pt_d.x,
                        lanes_combo[i].lanes[j].pts_ipm[k].pt_d.y);
            }

            //ff << std::endl;
            _fprintf(_fp, ";");
        }
    }

    _fprintf(_fp, "\n");

    for (int i = 0; i < arrows.size(); ++i) {
        for (int j = 0; j < arrows[i].pts_ipm.size(); ++j) {
            //ff << arrows[i].pts_ipm[j].x << " " << arrows[i].pts_ipm[j].y << " ";
            _fprintf(_fp, "%.20f,%.20f,", arrows[i].pts_ipm[j].x,
                    arrows[i].pts_ipm[j].y);
        }

        //ff << std::endl;
        _fprintf(_fp, ";");
    }

    _fprintf(_fp, "\n");
#endif

    if (!lanes_combo.size()) {
        return -1;
    }

    //for apollo *** std::cout << "        [-] hadmap offers " << lanes_combo[0].lanes.size() << " lanes among" << lanes_combo.size() << " combos" << std::endl;
    // for apollo ***   _timer.do_end_timer("    [-] End Requiring Hadmap Data, misc time: ");
    return 0;
}

int HadmapProcess::rotate_for_refinement(const cv::Point2d& center_pt, RETINFO& ret_info) {
    //for apollo *** std::cout << "    [+] Begin Rotating Hadmap Data" << std::endl;
    _timer.do_start_timer();
    // 4.1. fit all lane lines
    std::vector<float> angles_rot;
#ifndef SPEEDUP
    ret_info.im_hadmap = cv::Mat::zeros(im_h, im_w, CV_8U) + 32;
#endif
    std::vector<LANE_COMBO>& lanes_combo = ret_info.lanes_combo;
    std::vector<ARROW>& arrows = ret_info.arrows;

    for (int i = 0; i < lanes_combo.size(); ++i) {
        for (int j = 0; j < lanes_combo[i].lanes.size(); ++j) {
            // 4.1.1. from cm to pixel
            std::vector<WEIGHTEDPOINT>& pts_ipm = lanes_combo[i].lanes[j].pts_ipm;
            int sz = pts_ipm.size();

            if (pts_ipm.size() < 2) {
                lanes_combo[i].lanes.erase(lanes_combo[i].lanes.begin() + j);
                --j;
                std::cout << sz << " dianbugou" << std::endl;
                continue;
            }

            std::vector<cv::Point2f> pts(sz);

            for (int k = 0; k < sz; ++k) {
                pts_ipm[k].pt_d.x = (pts_ipm[k].pt_d.x - center_pt.x) * scale_x;
                pts_ipm[k].pt_d.y = (pts_ipm[k].pt_d.y - center_pt.y) * scale_y;
                //std::cout << pts_ipm[k].pt_d.x << " x  jianuq  y:" << pts_ipm[k].pt_d.y << std::endl;
                pts[k] = pts_ipm[k].pt_d;
            }

#ifndef SPEEDUP
            int interval = lanes_combo[i].lanes[j].type == 1 ? 1 : 4;

            for (int k = 0; k < sz - interval; k += interval) {
                cv::Point2f pt1 = pts_ipm[k].pt_d;
                cv::Point2f pt2 = pts_ipm[(k + 1) % sz].pt_d;
                pt1.x += 896;
                pt1.y += 896;
                pt2.x += 896;
                pt2.y += 896;
                //cv::circle(ret_info.im_hadmap, pt1, 5, cv::Scalar(200, 200, 200), -1);

                //by liuchao27 comment
                if (fabs(pt1.x - pt2.x) > 100
                        || fabs(pt1.y - pt2.y) > 100) {
                    continue;
                }

                cv::line(ret_info.im_hadmap, pt1, pt2, cv::Scalar(255), 4);
                //cv::imshow("sss", ret_info.im_hadmap);
                //cv::waitKey(0);
            }

#endif
            // 4.1.2. fitline and cal. the angle
            cv::fitLine(pts, lanes_combo[i].lanes[j].vec_ipm, CV_DIST_L2, 0, 0.01, 0.01);
            float alpha_rot = atan2f(lanes_combo[i].lanes[j].vec_ipm.val[0],
                                     lanes_combo[i].lanes[j].vec_ipm.val[1]);
            angles_rot.push_back(alpha_rot);
        }
    }

    //for apollo *** std::cout << "        [-] hadmap offers " << lanes_combo[0].lanes.size() << " lanes after pts size" << std::endl;
    //_timer.do_end_timer("    [-] first Rotating Hadmap Data, misc time: ");
    int sz = angles_rot.size();

    //std::cout << "sz11: " << sz << std::endl;
    if (!sz) {
        return -1;
    }

    // 4.2. rotate with the avg. angle
    // 4.2.1. calc. the avg. angle
    std::sort(angles_rot.begin(), angles_rot.end());
    float angle_rot = 0.0f;

    if (sz & 0x01) {
        angle_rot = angles_rot[(sz >> 1)];
    } else {
        angle_rot = (angles_rot[(sz >> 1)] + angles_rot[(sz >> 1) - 1]) / 2;
    }

    // 4.2.2. do the rotation
    for (int i = 0; i < lanes_combo.size(); ++i) {
        float max_len_tmp = 0.0f;
        lanes_combo[i].center_pt = cv::Point2f(0.0f, 0.0f);
        int count = 0;

        // 4.2.2.1. for each pt, rotate it
        for (int j = 0; j < lanes_combo[i].lanes.size(); ++j) {
            // 4.2.2.1.1. determine if add the CV_PI or not
            if (i == 0 && j == 0) {
                cv::Point2f pt0;
                cv::Point2d& pt0_d = lanes_combo[i].lanes[j].pts_ipm[0].pt_d;
                pt0.x = cos(angle_rot) * pt0_d.x - sin(angle_rot) * pt0_d.y;
                pt0.y = sin(angle_rot) * pt0_d.x + cos(angle_rot) * pt0_d.y;
                cv::Point2f pt1;
                cv::Point2d& pt1_d = lanes_combo[i].lanes[j].pts_ipm[1].pt_d;
                pt1.x = cos(angle_rot) * pt1_d.x - sin(angle_rot) * pt1_d.y;
                pt1.y = sin(angle_rot) * pt1_d.x + cos(angle_rot) * pt1_d.y;

                if (pt0.y > pt1.y) {
                    angle_rot += CV_PI;
                }
            }

            // 4.2.2.1.2. rotate it
            int sz_pts = lanes_combo[i].lanes[j].pts_ipm.size();

            for (int k = 0; k < sz_pts; ++k) {
                cv::Point2f& pt = lanes_combo[i].lanes[j].pts_ipm[k].pt;
                cv::Point2d& pt_d = lanes_combo[i].lanes[j].pts_ipm[k].pt_d;
                pt.x = cos(angle_rot) * pt_d.x - sin(angle_rot) * pt_d.y;
                pt.y = sin(angle_rot) * pt_d.x + cos(angle_rot) * pt_d.y;
                lanes_combo[i].center_pt += pt;
            }

            count += sz_pts;
        }

        lanes_combo[i].center_pt.x /= count;
        lanes_combo[i].center_pt.y /= count;
    }

    // 4.2.3. refine the pts
    //std::cout << "former angle size: " << angles_rot.size() << std::endl;
    for (int i = 0; i < lanes_combo[0].lanes.size(); ++i) {
        //std::cout << "dianshu: " << lanes_combo[0].lanes[i].pts_ipm.size() << std::endl;
        //for (int k = 0; k < lanes_combo[0].lanes[i].pts_ipm.size(); ++k)
        //{
        //    std::cout << lanes_combo[0].lanes[i].pts_ipm[k].pt.x << " pkg " << lanes_combo[0].lanes[i].pts_ipm[k].pt.y << std::endl;
        //}
    }

    angles_rot.clear();

    for (int i = 0; i < lanes_combo.size(); ++i) {
        // 4.2.2.1. for each pt, rotate it
        for (int j = 0; j < lanes_combo[i].lanes.size(); ++j) {
            // 4.2.2.1.3. kick out pts out of the image
            std::vector<cv::Point2f> pts;
            int min_y = INT_MAX;
            int max_y = INT_MIN;

            for (int k = 0; k < lanes_combo[i].lanes[j].pts_ipm.size(); ++k) {
                float y = _roi_out.height - lanes_combo[i].lanes[j].pts_ipm[k].pt.y * m_to_pixel;

                //std::cout << _roi_out.height << "  " << lanes_combo[i].lanes[j].pts_ipm[k].pt.y * m_to_pixel << "  ";
                //std::cout << y << std::endl;
                if (y < 0 ||
                        y >= _roi_out.height + 10) { //xhzhu 0823 +5 //+10
                    lanes_combo[i].lanes[j].pts_ipm.erase(lanes_combo[i].lanes[j].pts_ipm.begin()
                        + k);
                    --k;
                    //std::cout << "erase " << std::endl;;
                    continue;
                }

                if (y < min_y) {
                    min_y = y;
                }

                if (y > max_y) {
                    max_y = y;
                }
            }

            //              _timer.do_end_timer("    [-] 111, misc time: ");
            //std::cout << "1111 tichu: " << lanes_combo[i].lanes[j].pts_ipm.size() << std::endl;
            for (int k = 0; k < lanes_combo[i].lanes[j].pts_ipm.size(); ++k) {
                float y = _roi_out.height - lanes_combo[i].lanes[j].pts_ipm[k].pt.y * m_to_pixel;

                if (y >= max_y - _roi_out.height * 1 / 4) {
                    pts.push_back(cv::Point2f(lanes_combo[i].lanes[j].pts_ipm[k].pt.x,
                                              lanes_combo[i].lanes[j].pts_ipm[k].pt.y * 100));
                }
            }

            //std::cout << "2222 pts: " << pts.size() << std::endl;

            if (lanes_combo[i].lanes[j].pts_ipm.size() < 2) {
                lanes_combo[i].lanes.erase(lanes_combo[i].lanes.begin() + j);
                --j;
                //std::cout << "delete lane " << j << std::endl;
                continue;
            }

            /*
            if (min_y >= _roi_out.height * 4/5)
                 {
                                         std::cout << "bad min_y" << std::endl;
                     continue;
                 }
            */

            if (pts.size() >= 2) {
                cv::fitLine(pts, lanes_combo[i].lanes[j].vec_ipm, CV_DIST_L2, 0, 0.01, 0.01);
                float alpha_rot = atan2f(lanes_combo[i].lanes[j].vec_ipm.val[0],
                                         lanes_combo[i].lanes[j].vec_ipm.val[1] / 100);

                if (fabs(alpha_rot) > CV_PI / 2) {
                    alpha_rot -= CV_PI;
                }

                for (int iter = 0; iter < lanes_combo[i].lanes[j].pts_ipm.size(); ++iter) {
                    angles_rot.push_back(alpha_rot);
                }
            }
        }

        if (lanes_combo[i].lanes.size() == 0) {
            lanes_combo.erase(lanes_combo.begin() + i);
            --i;
            continue;
        }
    }

    //for apollo *** std::cout << "        [-] hadmap offers " << lanes_combo[0].lanes.size() << " lanes after height region" << std::endl;
    //_timer.do_end_timer("    [-] second Rotating Hadmap Data, misc time: ");
    sz = angles_rot.size();

    if (!sz) {
        return -1;
    }

    // 4.2. rotate with the avg. angle
    // 4.2.1. calc. the avg. angle
    std::sort(angles_rot.begin(), angles_rot.end());
    float angle_rot_refine = 0.0f;

    if (sz & 0x01) {
        angle_rot_refine = angles_rot[(sz >> 1)];
    } else {
        angle_rot_refine = (angles_rot[(sz >> 1)] + angles_rot[(sz >> 1) - 1]) / 2;
    }

    _angle = angle_rot + angle_rot_refine;
    ret_info.rotate_angle = _angle;

    //std::cout << "angle " << _angle << " = " << angle_rot << " + " << angle_rot_refine << std::endl;

    // 4.2.2. do the rotation
    for (int i = 0; i < lanes_combo.size(); ++i) {
        float max_len_tmp = 0.0f;
        lanes_combo[i].center_pt = cv::Point2f(0.0f, 0.0f);
        int count = 0;

        // 4.2.2.1. for each pt, rotate it
        for (int j = 0; j < lanes_combo[i].lanes.size(); ++j) {
            // 4.2.2.1.2. rotate it
            int sz_pts = lanes_combo[i].lanes[j].pts_ipm.size();

            for (int k = 0; k < sz_pts; ++k) {
                cv::Point2f pt = lanes_combo[i].lanes[j].pts_ipm[k].pt;
                lanes_combo[i].lanes[j].pts_ipm[k].pt.x = cos(angle_rot_refine) * pt.x - sin(
                            angle_rot_refine) * pt.y + shift_x;
                lanes_combo[i].lanes[j].pts_ipm[k].pt.y = _roi_out.height - 
                            (sin(angle_rot_refine) * pt.x + cos(
                            angle_rot_refine) * pt.y) * m_to_pixel;
            }
        }
    }

    // 4.2.3. refine the pts
    float max_len = 0.0f;

    for (int i = 0; i < lanes_combo.size(); ++i) {
        float max_len_tmp = 0.0f;

        // 4.2.2.1. for each pt, rotate it
        for (int j = 0; j < lanes_combo[i].lanes.size(); ++j) {
            std::sort(lanes_combo[i].lanes[j].pts_ipm.begin(), \
                      lanes_combo[i].lanes[j].pts_ipm.end(), sort_by_y_asc);
            // 4.2.2.1.4. fix the start and end pt
            lanes_combo[i].lanes[j].start_pt_ipm = lanes_combo[i].lanes[j].pts_ipm[0].pt;
            lanes_combo[i].lanes[j].end_pt_ipm =
                lanes_combo[i].lanes[j].pts_ipm[lanes_combo[i].lanes[j].pts_ipm.size() - 1].pt;
            // 4.2.2.1.5. judge if the lane line is an assisted one
            float alpha_tmp = atan2f(lanes_combo[i].lanes[j].end_pt_ipm.y -
                                     lanes_combo[i].lanes[j].start_pt_ipm.y,
                                     lanes_combo[i].lanes[j].end_pt_ipm.x 
                                     - lanes_combo[i].lanes[j].start_pt_ipm.x);
            alpha_tmp = alpha_tmp < 0 ? alpha_tmp + CV_PI : alpha_tmp;

            if (fabs(alpha_tmp - CV_PI / 2) > CV_PI * 5 / 180) {
                lanes_combo[i].lanes[j].is_assisted_one = true;
            } else {
                lanes_combo[i].lanes[j].is_assisted_one = false;
            }

            float tmp = lanes_combo[i].lanes[j].start_pt_ipm.y;

            if (tmp < _roi_out.height * 3 / 4
                    && tmp > max_len_tmp) {
                max_len_tmp = tmp;
            }
        }

        std::sort(lanes_combo[i].lanes.begin(), lanes_combo[i].lanes.end(), sort_by_x_asc);

        // 4.2. if the len is larger, erase the front lanes
        if (max_len_tmp > max_len) {
            if (i != 0) {
                std::swap(lanes_combo[0], lanes_combo[i]);
            }

            //_lanes_combos.erase(_lanes_combos.begin(), _lanes_combos.begin() + i);
            max_len = max_len_tmp;
            //--i;
            //continue;
        }
    }

    for (int i = 0; i < arrows.size(); ++i) {
        int sz = arrows[i].pts_ipm.size();

        for (int j = 0; j < sz; ++j) {
            cv::Point2d pt = arrows[i].pts_ipm[j];
            arrows[i].pts_ipm[j].x = (pt.x - center_pt.x) * scale_x;
            arrows[i].pts_ipm[j].y = (pt.y - center_pt.y) * scale_y;
        }

#ifndef SPEEDUP

        for (int j = 0; j < sz; ++j) {
            cv::Point2f pt1 = arrows[i].pts_ipm[j];
            cv::Point2f pt2 = arrows[i].pts_ipm[(j + 1) % sz];
            pt1.x += 896;
            pt1.y += 896;
            pt2.x += 896;
            pt2.y += 896;

            if (fabs(pt1.x - pt2.x) > 200
                    || fabs(pt1.y - pt2.y) > 200) {
                continue;
            }

            cv::line(ret_info.im_hadmap, pt1, pt2, cv::Scalar(128), 3);
            //          cv::imshow("show", ret_info.im_hadmap);
            //          cv::waitKey(0);
        }

#endif
        float min_y = INT_MAX;
        float min_y2 = INT_MAX;
        float max_y = INT_MIN;
        float max_y2 = INT_MIN;

        for (int j = 0; j < sz; ++j) {
            cv::Point2d pt = arrows[i].pts_ipm[j];
            arrows[i].pts_ipm[j].x = cos(_angle) * pt.x - sin(_angle) * pt.y + shift_x;
            arrows[i].pts_ipm[j].y = _roi_out.height - (sin(_angle) 
                                    * pt.x + cos(_angle) * pt.y) * m_to_pixel;

            if (arrows[i].pts_ipm[j].y < min_y) {
                min_y = arrows[i].pts_ipm[j].y;
            } else {
                if (arrows[i].pts_ipm[j].y < min_y2) {
                    min_y2 = arrows[i].pts_ipm[j].y;
                }
            }

            if (arrows[i].pts_ipm[j].y > max_y) {
                max_y = arrows[i].pts_ipm[j].y;
            } else {
                if (arrows[i].pts_ipm[j].y > max_y2) {
                    max_y2 = arrows[i].pts_ipm[j].y;
                }
            }
        }

        if (fabs(max_y2 - max_y) < 5) {
            arrows[i].bottom_line = (max_y + max_y2) / 2;
        } else {
            arrows[i].bottom_line = max_y;
        }

        if (fabs(min_y2 - min_y) < 5) {
            arrows[i].top_line = (min_y + min_y2) / 2;
        } else {
            arrows[i].top_line = min_y;
        }
    }

    //#endif
    //cv::imwrite("sb.png", ret_info.im_hadmap);
    //cv::Point2d pt = _ret_info.pt_ground_truth;
    //_ret_info.pt_ground_truth.x = cos(_angle) * pt.x - sin(_angle) * pt.y + shift_x;
    //_ret_info.pt_ground_truth.y = _roi_out.height - ((sin(_angle) * pt.x + cos(_angle) * pt.y) / 4);
#ifndef SPEEDUP
    cv::flip(ret_info.im_hadmap, ret_info.im_hadmap, 0);
    //cv::imshow("sb", ret_info.im_hadmap);
    //cv::waitKey(0);
#endif
    //for apollo ***    _timer.do_end_timer("    [-] End Rotating Hadmap Data, misc time: ");
    //_ret_info.im_hadmap = _ret_info.im_hadmap(cv::Rect(448, 448, 896, 896)).clone();
    //cv::flip(im_draw, im_draw, 0);
    //cv::imwrite("sb.png", im_draw);
    //imshow("sbb", im_draw);
    //cv::waitKey(0);
    return 0;
}

SYSTEM_STATUS HadmapProcess::do_query_hadmap(
    const cv::Vec3d& gps,
    DYNAMIC_PARAMS& para,
    RETINFO& ret_info) {
    ret_info.lanes_combo.clear();
    ret_info.arrows.clear();
    // 7. compare with the hadmap
    int flag;
    flag = require_hadmap_data(gps, ret_info);

    //illegal
    if (flag != 0) {
        std::cout << "bibishdjh 1111111111" << std::endl;
        return SYS_ERR;
    }

    _lanes_combo = ret_info.lanes_combo;
    _arrows = ret_info.arrows;
    flag = rotate_for_refinement(_initial_center_pt, ret_info);

    //xhzhu 0822
    if (flag != 0) {
        std::cout << "nlkjlrj 2222222222222" << std::endl;
        return SYS_ERR;
    }

    double scale = 50;

    if (_old_gt.size() == 2) {
        scale = sqrt((_old_gt[1].x - _old_gt[0].x) * (_old_gt[1].x - _old_gt[0].x) 
                    * scale_x * scale_x
                     + (_old_gt[1].y - _old_gt[0].y) * (_old_gt[1].y - _old_gt[0].y) 
                     * scale_y * scale_y)/* * 1.03*/;
    }

    para.vo_scale = scale;
    return SYS_SCENE_PARSE;
}

SYSTEM_STATUS HadmapProcess::do_delta_y_estimation(
    RETINFO& ret_info) {
    //for apollo *** std::cout << "    [+] Begin Delta-y Estimation" << std::endl;
    _timer.do_start_timer();
    // 1. if delta_y has been calculated
    //bool flag = false;
    cv::Point2d old_pos_in_new_axis;
    cv::Point2d pt_gt22;

    if (_old_estimations.size() > 0) {
        cv::Point2f pt;
        pt.x = (_old_estimations[_old_estimations.size() - 1].x - _initial_center_pt.x) * scale_x;
        pt.y = (_old_estimations[_old_estimations.size() - 1].y - _initial_center_pt.y) * scale_y;
        //cv::Point2d pt_gt2;
        old_pos_in_new_axis.x = cos(_angle) * pt.x - sin(_angle) * pt.y + shift_x;
        old_pos_in_new_axis.y = -(sin(_angle) * pt.x + cos(_angle) * pt.y);
        pt.x = (_old_gt[0].x - _initial_center_pt.x) * scale_x;
        pt.y = (_old_gt[0].y - _initial_center_pt.y) * scale_y;
        pt_gt22.x = cos(_angle) * pt.x - sin(_angle) * pt.y + shift_x;
        pt_gt22.y = -(sin(_angle) * pt.x + cos(_angle) * pt.y);
    }

    // 2. update the kalman filter
    std::vector<float> kalman_outputs;

    if (_old_estimations.size() >= 1) {
        if (NULL == _kalman_position_y) {
            _kalman_position_y = new Kalman2;
            std::vector<float> measure_vals;
            _old_shift_y = old_pos_in_new_axis.y;
            measure_vals.push_back(_old_shift_y);
            _kalman_position_y->init(1, 2, measure_vals, 1e-3, 1); //xhzhu 0824
            kalman_outputs.push_back(_old_shift_y);
            kalman_outputs.push_back(_old_shift_y);
            //kalman_outputs.push_back(0);
        } else {
            // 4.2. refine the kalman
            // e.g., the old_pos = 104, shift = -99, the estimation.y = 5
            // we should use 99 for output, but use 104 for update
            std::vector<float> measure_vals;
            measure_vals.push_back(_old_shift_y + old_pos_in_new_axis.y);
            //_old_shift_y += old_pos_in_new_axis.y;
            _kalman_position_y->do_update(measure_vals);
            // 1.1.2. kalman output the new s, should - the old s
            // calc the shift with the kalman filter
            // e.g., the new s = 10000, old s = 9901, shift = 99
            kalman_outputs = _kalman_position_y->do_predict();
            ret_info.motion_state.val[0] = kalman_outputs[0];
            ret_info.motion_state.val[1] = kalman_outputs[1];
            //ret_info.motion_state.val[2] = kalman_outputs[2];
            kalman_outputs[0] -= _old_shift_y;
            _old_shift_y += kalman_outputs[0];
        }
    }

    //_timer.do_end_timer("    [-] kalman, misc time: ");

    if (ret_info.pt_estimation_level.y == 100) {
        // 1.1. if has slam, use slam, not, will use kalman
        if (_old_estimations.size() >= 1) {
            ret_info.pt_estimation_draw.y = -old_pos_in_new_axis.y + kalman_outputs[0];
            ret_info.pt_estimation_level.y = 2;
        } else {
            ret_info.pt_estimation_draw.y = 0;
            ret_info.pt_estimation_level.y = 3;
        }
    } else {
        ret_info.pt_estimation_draw.y = (-ret_info.pt_estimation_draw.y) / m_to_pixel;
        //flag = true;
    }

    //// 2. just for check with gt
    //cv::Point2d pt_gt = ret_info.pt_ground_truth;
    //cv::Point2d pt_gt2;
    //pt_gt2.x = cos(_angle) * pt_gt.x - sin(_angle) * pt_gt.y + shift_x;
    //pt_gt2.y = -(sin(_angle) * pt_gt.x + cos(_angle) * pt_gt.y);
    //ret_info.pt_error.x = ret_info.pt_estimation.x - pt_gt2.x;
    //ret_info.pt_error.y = -ret_info.pt_estimation.y - pt_gt2.y;
    //cv::Point2d pt_gps = ret_info.pt_gps;
    //cv::Point2d pt_gps2;
    //pt_gps2.x = cos(_angle) * pt_gps.x - sin(_angle) * pt_gps.y + shift_x;
    //pt_gps2.y = -(sin(_angle) * pt_gps.x + cos(_angle) * pt_gps.y);
    //ret_info.pt_error_gps.x = pt_gps2.x - pt_gt2.x;
    //ret_info.pt_error_gps.y = pt_gps2.y - pt_gt2.y;
    // 3. prepare the final position
    cv::Point2d pt = ret_info.pt_estimation_draw;
    pt.x = pt.x - shift_x;
    ret_info.pt_estimation_draw.x = cos(_angle) * pt.x + sin(_angle) * pt.y;
    ret_info.pt_estimation_draw.y = -sin(_angle) * pt.x + cos(_angle) * pt.y;
    cv::Point2d pt_final;
    pt_final.x = ret_info.pt_estimation_draw.x / scale_x + _initial_center_pt.x;
    pt_final.y = ret_info.pt_estimation_draw.y / scale_y + _initial_center_pt.y;
    //_timer.do_end_timer("    [-] End Delta-y Estimation, misc time: ");
    _refined_y_center_pt = pt_final;
    ret_info.lanes_combo = _lanes_combo;
    ret_info.arrows = _arrows;
    rotate_for_refinement(_refined_y_center_pt, ret_info);
    //_timer.do_end_timer("    [-] 4444444, misc time: ");
    ret_info.pt_estimation_draw.y = 0;
    ret_info.initial_center = _initial_center_pt; //xhzhu 0808 lane keep
    return SYS_GROUND_MATCH_X;
}

SYSTEM_STATUS HadmapProcess::do_delta_x_estimation(
    RETINFO& ret_info) {
    //for apollo *** std::cout << "    [+] Begin Delta-x Estimation" << std::endl;
    _timer.do_start_timer();
    cv::Point pt_estimation_level_old = ret_info.pt_estimation_level;
    cv::Point2f pt_estimation_draw_old = ret_info.pt_estimation_draw;
    // 2. short-term correction
    int sz = _pt_estimations.size();

    //std::cout << "short start o " << sz  << std::endl;
    if (sz >= valid_deque) {
        // 2.1. use old record for line fitting
#ifndef SPEEDUP
        FILE* _fp = fopen("sb.txt", "w");
#endif
        std::vector<WEIGHTEDPOINT> pts;

        for (int i = sz - 1; i >= 0; --i) {
            if (_pt_estimations[i].level >= 2) {
                continue;
            }

            cv::Point2f pt;
            pt.x = (_pt_estimations[i].pt.x - _refined_y_center_pt.x) * scale_x;
            pt.y = (_pt_estimations[i].pt.y - _refined_y_center_pt.y) * scale_y;
            cv::Point2d pt_gt2;
            pt_gt2.x = cos(_angle) * pt.x - sin(_angle) * pt.y + shift_x;
            pt_gt2.y = -(sin(_angle) * pt.x + cos(_angle) * pt.y);
            WEIGHTEDPOINT pt_weighted;
            pt_weighted.pt = cv::Point2f(pt_gt2.y * 100,
                                         pt_gt2.x); 
            // tips: if y is closer, may not fitted, so use constant interval
            //pt_weighted.weight = (i + valid_deque - sz) * 5 + 1;
            pts.push_back(pt_weighted);
#ifndef SPEEDUP
            _fprintf(_fp, "%.20f %.20f %.20f\n", pt_gt2.x, pt_gt2.y, i);
#endif
        }

        //std::cout << "xuandian" << std::endl;
#ifndef SPEEDUP
        fclose(_fp);
#endif

        if (pts.size() >= valid_deque) {
            pts.erase(pts.begin() + valid_deque, pts.end());

            for (int i = 0; i < valid_deque; ++i) {
                pts[i].weight = (valid_deque - 1 - i) * 5 + 1;
            }

            cv::Vec4f vec;
            //std::cout << "zuodian ransac" << std::endl;
            std::vector<WEIGHTEDPOINT> pts_ransac = 
                _ransac_fitting.ransac_fit_poly_weighted(pts, vec);
            //std::cout << "zuowan ransac" << std::endl;
            cv::Point2d pt_estimation_fitted;
            //std::cout << " jiancha vec::   " << vec[2] << std::endl;
            pt_estimation_fitted.x = vec[2];
            //std::cout << pt_estimation_fitted.x << std::endl;
            //xhzhu 0823
            // as may have big error in y ,so for ransac predict may result in big error in x too. Add some limit.
            std::cout << pt_estimation_fitted.x << "  " << pts[0].pt.y << std::endl;
            ret_info.short_x = pt_estimation_fitted.x;
            //          if (abs(pt_estimation_fitted.x - pts[0].pt.y) > 4) {
            //              pt_estimation_fitted.x = pts[0].pt.y + (pt_estimation_fitted.x > pts[0].pt.y?1:-1) * 4;
            //          }
            // 2.2.
            // is estimate and fit is smaller than 40cm, use estimate
            // if estimate and fit is larger than 40cm, use fit;
            // here, if estimate and fit is larger than 100cm, the estimate cannot be believed
            std::cout << "        [-] short term: estimate x = "
                    << ret_info.pt_estimation_draw.x <<
                    ", fitted x = " << pt_estimation_fitted.x << std::endl;
            //std::cout <<  " shuchu:  " << pts_ransac.size() << "   " << ret_info.pt_estimation_draw.x << std::endl;
            //0827 xhzhu if very bad, use it forcely.
            //if (fabs(pt_estimation_fitted.x - ret_info.pt_estimation_draw.x) > 60)
            //{
            //    ret_info.pt_estimation_level.x = 2;
            //    ret_info.pt_estimation_draw.x = pt_estimation_fitted.x;
            //}

            if (pts_ransac.size() > (valid_deque * 0.8)) { //xhzhu 0814 0.8
                float diff = fabs(pt_estimation_fitted.x - ret_info.pt_estimation_draw.x);

                if (diff > 20) {
                    //ret_info.pt_estimation_draw.x = pt_estimation_fitted.x;
                    //ret_info.pt_estimation_level.x = 100;
                    //pt_estimation_draw_old.x = pt_estimation_fitted.x;
                    //pt_estimation_level_old.x = 2;
                } else if (diff > 8) //xhzhu 0823 diff > 8
                    //if (diff > 8)
                {
                    ret_info.pt_estimation_level.x = 2;
                    ret_info.pt_estimation_draw.x = pt_estimation_fitted.x;
                    //xhzhu 0827 reverse use
                    //  pt_estimation_draw_old.x = pt_estimation_fitted.x;
                    //  pt_estimation_level_old.x = 2;
                    std::cout << "use ransac " << std::endl;
                }

                //std::cout << "%%%% short 1 " << std::endl;
            }

            //0827 xhzhu if very bad, use it forcely.
            //else if (fabs(pt_estimation_fitted.x - ret_info.pt_estimation_draw.x) > 40)
            //{
            //    ret_info.pt_estimation_level.x = 2;
            //    ret_info.pt_estimation_draw.x = pt_estimation_fitted.x;
            //}
        }
    }

    std::cout << "short ransac draw x: " << ret_info.pt_estimation_draw.x << std::endl;
    //std::cout << "ganwanle" << std::endl;

    // 3. long-term correction (if have short_term correction, no long-term needed)
    if (sz >= max_deque) {
        // 2.1. use old record for line fitting
        std::vector<WEIGHTEDPOINT> pts;

        for (int i = 0; i < sz; ++i) {
            //xhzhu 0907
            if (_pt_estimations[i].level >= 2) {
                continue;
            }

            cv::Point2f pt;
            pt.x = (_pt_estimations[i].pt.x - _refined_y_center_pt.x) * scale_x;
            pt.y = (_pt_estimations[i].pt.y - _refined_y_center_pt.y) * scale_y;
            cv::Point2d pt_gt2;
            pt_gt2.x = cos(_angle) * pt.x - sin(_angle) * pt.y + shift_x;
            pt_gt2.y = -(sin(_angle) * pt.x + cos(_angle) * pt.y);
            WEIGHTEDPOINT pt_weighted;
            pt_weighted.pt = cv::Point2f(pt_gt2.y * 100,
                                         pt_gt2.x); 
            // tips: if y is closer, may not fitted, so use constant interval
            pt_weighted.weight = (i + max_deque - sz) * 5 + 1;
            pts.push_back(pt_weighted);
        }

        if (pts.size() >= max_deque) {
            int sz_max = pts.size();
            pts.erase(pts.begin(), pts.begin() + sz_max - max_deque);

            for (int i = 0; i < max_deque; ++i) {
                pts[i].weight = i * 5 + 1;
            }

            cv::Vec4f vec;
            std::vector<WEIGHTEDPOINT> pts_ransac = 
                _ransac_fitting.ransac_fit_poly_weighted(pts, vec);
            cv::Point2d pt_estimation_fitted;
            pt_estimation_fitted.x = vec[2];
            //std::cout << pt_estimation_fitted.x << std::endl;
            // 2.2.
            // is estimate and fit is smaller than 30cm, use estimate
            // if estimate and fit is larger than 30cm, use fit;
            // here, if estimate and fit is larger than 100cm, the estimate cannot be believed
            std::cout << "        [-] long term: estimate x = " << ret_info.pt_estimation_draw.x <<
                      ", fitted x = " << pt_estimation_fitted.x << std::endl;
            ret_info.long_x = pt_estimation_fitted.x;

            /*   //xhzhu 0827 for better use of short
            if (pts_ransac.size() > (max_deque * 0.8)) //0824 0.8
                 {
                     float diff = fabs(pt_estimation_fitted.x - ret_info.pt_estimation_draw.x);
                     float diff2 = fabs(pt_estimation_fitted.x - pt_estimation_draw_old.x);

                     // if have short correction and short correction is right, use it
                     //std::cout << "diff: " << diff <<  "  " << diff2 << std::endl;
                                         if (pt_estimation_level_old.x == 2
                         && diff2 <= 8) //xhzhu 0814 8
                     {
                                                 //std::cout << "choice 1" << std::endl;
                         ret_info.pt_estimation_level.x = 2;
                         ret_info.pt_estimation_draw.x = pt_estimation_draw_old.x;
                     }
                     else if (diff > 40) //xhzhu 0814 40
                     {
                                                 //std::cout << "choice 2" << std::endl;
                         ret_info.pt_estimation_level.x = 3;
                         ret_info.pt_estimation_draw.x = pt_estimation_fitted.x;

                         pt_estimation_draw_old.x = pt_estimation_fitted.x;
                         pt_estimation_level_old.x = 3;
                     }
                                         //std::cout << "&&&& long 2" << std::endl;
                 }
                                 */
            if (pts_ransac.size() > (max_deque * 0.8)) { //0824 0.8
                float diff2 = fabs(pt_estimation_fitted.x - ret_info.pt_estimation_draw.x);
                float diff = fabs(pt_estimation_fitted.x - pt_estimation_draw_old.x);

                // if have short correction and short correction is right, use it
                //std::cout << "diff: " << diff <<  "  " << diff2 << std::endl;
                if (ret_info.pt_estimation_level.x == 2
                        && diff2 <= 8) { //xhzhu 0814 8
                    //std::cout << "choice 1" << std::endl;
                    //ret_info.pt_estimation_level.x = 2;
                    //ret_info.pt_estimation_draw.x = pt_estimation_draw_old.x;
                } else if (diff > 40) { //xhzhu 0814 40
                    //std::cout << "choice 2" << std::endl;
                    ret_info.pt_estimation_level.x = 3;
                    ret_info.pt_estimation_draw.x = pt_estimation_fitted.x;
                    pt_estimation_draw_old.x = pt_estimation_fitted.x;
                    pt_estimation_level_old.x = 3;
                } else { // get back estimation
                    ret_info.pt_estimation_level.x = pt_estimation_level_old.x;
                    ret_info.pt_estimation_draw.x = pt_estimation_draw_old.x;
                }

                //std::cout << "&&&& long 2" << std::endl;
            }
        }
    }

    std::cout << ret_info.pt_estimation_draw.x << std::endl;
    //ret_info.final_x = ret_info.pt_estimation_draw.x;

    if (ret_info.pt_estimation_level.x == 100) {
        ret_info.pt_estimation_draw.x = shift_x;
        ret_info.pt_estimation_level.x = 4;
    }

    //std::cout << "before erase "<<  _old_gt.size() << std::endl;
    _old_gt.push_back(ret_info.pt_ground_truth_draw);

    if (_old_gt.size() > 2) {
        _old_gt.erase(_old_gt.begin(), _old_gt.end() - 2);
    }

    //std::cout << "erase end "<< _old_gt.size()  << std::endl;
    ret_info.pt_ground_truth_draw.x = (ret_info.pt_ground_truth_draw.x - _initial_center_pt.x) *
                                      scale_x;
    ret_info.pt_ground_truth_draw.y = (ret_info.pt_ground_truth_draw.y - _initial_center_pt.y) *
                                      scale_y;
    ret_info.pt_gps_draw.x = 0;
    ret_info.pt_gps_draw.y = 0;
    // 2. just for check with gt
    cv::Point2d pt_gt = ret_info.pt_ground_truth_draw;
    cv::Point2d pt_gt2;
    pt_gt2.x = cos(_angle) * pt_gt.x - sin(_angle) * pt_gt.y + shift_x;
    pt_gt2.y = -(sin(_angle) * pt_gt.x + cos(_angle) * pt_gt.y);
    //cv::Point2d pt_gps = ret_info.pt_gps;
    //cv::Point2d pt_gps2;
    //pt_gps2.x = cos(_angle) * pt_gps.x - sin(_angle) * pt_gps.y + shift_x;
    //pt_gps2.y = -(sin(_angle) * pt_gps.x + cos(_angle) * pt_gps.y);
    //ret_info.pt_error_gps.x = pt_gps2.x - pt_gt2.x;
    //ret_info.pt_error_gps.y = pt_gps2.y - pt_gt2.y;
    // 3. prepare the final position
    cv::Point2d pt = ret_info.pt_estimation_draw;
    //for data 07.07
    //pt.x = pt.x - shift_x - 14;//apollo trick + 5;
    //pt.y = pt.y - 1;//apollo trick + 50;
    //std::cout << _judge_outer << " _is_outer " << std::endl;
    /*
            if (_judge_outer == "outer") //xhzhu 0821
            {
                pt.x = pt.x - shift_x + 24 + 7.5 + 6.3; //2(07.14) xhzhu 0811 //0815 inner zai +18
                pt.y = pt.y + 39 - 34 - 6.9; // inner zai -5.568
            }
            else
            {
                pt.x = pt.x - shift_x + 24 + 18 - 24 - 24; //2(07.14) xhzhu 0811 //0815 inner zai +18 //0824 -24
                pt.y = pt.y - 1;
            }
    */
    //0822
    //pt.x = pt.x - shift_x + 24 + 7.5 - 6.4 - 1.2; //2(07.14) xhzhu 0811 //0815 inner zai +18
    //pt.y = pt.y; //+ 39 - 34 - 6.9 + 29 + 11.8;
    //0916 adaptive
    float r_ang = ret_info.yaw_filtered; //+ 1.576;
    int length_cpt_cam = 8 * 16;
    //std::cout << "aaaaangle: " << r_ang << std::endl;
    //std::cout << "r_ang:  " << r_ang << std::endl;
    //for inner estimation 0918
    cv::Point2d pt_in;
    pt_in.x = pt.x - shift_x;
    pt_in.y = pt.y;
    pt.x = pt.x - shift_x + length_cpt_cam * sin((r_ang) / 180 * 3.141593) - 0;
    pt.y = pt.y; //+ length_cpt_cam * cos((r_ang-3) / 140 * 3.141593);
    ret_info.final_x = pt.x + shift_x;

    // !! 0919 we get final pt in direct forward axis, if not any change, this point will
    //be transform into final gps. so for the outer final filter, the final check for eliminating
    // a little rate of interupt deviation should be done here using former estimation results.
    /*if (_pts_outer_filter.size() >= 1) {
        std::cout << _pts_outer_filter[0].x << "\t" << _pts_outer_filter[0].y << "\t" <<
                  _refined_y_center_pt.x << "\t" << _refined_y_center_pt.y << std::endl;
        cv::Point2d pt_01;
        pt_01.x = (_pts_outer_filter[0].x - _refined_y_center_pt.x) * scale_x;
        pt_01.y = (_pts_outer_filter[0].y - _refined_y_center_pt.y) * scale_y;
        std::cout << pt_01.x << "\t" << pt_01.y << std::endl;
        cv::Point2d pt_02;
        pt_02.x = cos(_angle) * pt_01.x - sin(_angle) * pt_01.y;
        pt_02.y = -(sin(_angle) * pt_01.x + cos(_angle) * pt_01.y);
        std::cout << "check outer lvboooo: " << pt.x << "\t" << pt.y 
                << "\tformer: " << pt_02.x << "\t" << pt_02.y << std::endl;
        double lateral_walk_step = 0.3;

        if (fabs(pt.x - pt_02.x) > 40) {
            pt.x = pt.x > pt_02.x ? pt_02.x + lateral_walk_step : pt_02.x - lateral_walk_step;
        }

        std::cout << "check outer lvboooo: " << pt.x << "\t" << pt.y 
                    << "\tformer: " << pt_02.x << "\t" << pt_02.y << std::endl;
    }*/
    
    //std::cout << "delts_x_estimation" << std::endl;
    ret_info.pt_estimation_draw.x = cos(_angle) * pt.x + sin(_angle) * pt.y;
    ret_info.pt_estimation_draw.y = -sin(_angle) * pt.x + cos(_angle) * pt.y;
    //cv::Point2d pt_draw_in; //inner 0918
    //pt_draw_in.x = cos(_angle) * pt_in.x + sin(_angle) * pt_in.y;
    //pt_draw_in.y = -sin(_angle) * pt_in.x + cos(_angle) * pt_in.y;
    pt = pt_estimation_draw_old;
    pt.x = pt_estimation_draw_old.x - shift_x;
    pt_estimation_draw_old.x = cos(_angle) * pt.x + sin(_angle) * pt.y;
    pt_estimation_draw_old.y = -sin(_angle) * pt.x + cos(_angle) * pt.y;
    cv::Point2d pt_final;
    pt_final.x = ret_info.pt_estimation_draw.x / scale_x + _refined_y_center_pt.x;
    pt_final.y = ret_info.pt_estimation_draw.y / scale_y + _refined_y_center_pt.y;
#ifdef SPEEDUP
    ret_info.pt_estimation_final = _hadmap_query.mercator_to_lonlat(pt_final);
#else
    ret_info.pt_estimation_final = pt_final;
#endif
    //for apollo *** std::cout << "        [-] ### refined pos = " << std::setprecision(16) << ret_info.pt_estimation_final << " ###" << std::endl;
#ifdef OUTPUT_TXT
    _fprintf(_fp, "%.20f,%.20f\n", pt_final.x,
            pt_final.y);
#endif
    ret_info.pt_estimation_draw.x = (pt_final.x - _initial_center_pt.x) * scale_x;
    ret_info.pt_estimation_draw.y = (pt_final.y - _initial_center_pt.y) * scale_y;
    cv::Point2d pt_final_old;
    pt_final_old.x = pt_estimation_draw_old.x / scale_x + _refined_y_center_pt.x;
    pt_final_old.y = pt_estimation_draw_old.y / scale_y + _refined_y_center_pt.y;
    pt_estimation_draw_old.x = (pt_final_old.x - _initial_center_pt.x) * scale_x;
    pt_estimation_draw_old.y = (pt_final_old.y - _initial_center_pt.y) * scale_y;
    OLD_RECORD record;
    record.pt = pt_final_old;
    record.level = pt_estimation_level_old.x;
    _pt_estimations.push_back(record);

    if (_pt_estimations.size() > 50) {
        _pt_estimations.pop_front();
    }

    _old_estimations.push_back(pt_final_old);

    if (_old_estimations.size() > 1) {
        _old_estimations.pop_front();
    }

    _pts_outer_filter.push_back(pt_final);

    if (_pts_outer_filter.size() > 1) {
        _pts_outer_filter.pop_front();
    }

    //if (ret_info.pt_estimation_level.x < 3)
    //{
    //  _old_estimations.push_back(pt_final);
    //  if (_old_estimations.size() > 2)
    //  {
    //      _old_estimations.pop_front();
    //  }
    //  if (NULL == _kalman_position_y)
    //  {
    //      // 4.1. if v and s have been assigned, kalman can be initialized
    //      // here estimation.y = 0, if old_pos_in_new_axis.y = 101, v and s are both 101
    //      if (_old_estimations.size() == 2)
    //      {
    //          _kalman_position_y = new Kalman2;
    //          std::vector<float> measure_vals;
    //          _old_shift_y = old_pos_in_new_axis.y/* - ret_info.pt_estimation.y*/;
    //          measure_vals.push_back(_old_shift_y);
    //          _kalman_position_y->init(1, 2, measure_vals, 1e-5, 1e-2);
    //      }
    //
    //  }
    //  else
    //  {
    //      // 4.2. refine the kalman
    //      // e.g., the old_pos = 104, shift = -99, the estimation.y = 5
    //      // we should use 99 for output, but use 104 for update
    //      std::vector<float> measure_vals;
    //      measure_vals.push_back(_old_shift_y + old_pos_in_new_axis.y);
    //      _old_shift_y += old_pos_in_new_axis.y;
    //      _kalman_position_y->do_update(measure_vals);
    //  }
    //
    //}
    // 4. if level-0 results, record the pos
#ifndef __GNUC__

    if (ret_info.pt_estimation_level.y == 0) {
        POS pos;
        pos.loc = pt_final;
        pos.angle = _angle;
        _traces.push_back(pos);
        _slam_lasting = 0;
    }

#endif
    ret_info.angle = _angle;
    //ret_info.pt_estimation_draw.x -= 3; //xhzhu 7.11
    //ret_info.pt_estimation_draw.y -= 1;
    cv::Point2d pt_estimation2;
    pt_estimation2.x = cos(_angle) * ret_info.pt_estimation_draw.x - sin(_angle) *
                       ret_info.pt_estimation_draw.y + shift_x;
    pt_estimation2.y = -(sin(_angle) * ret_info.pt_estimation_draw.x + cos(
                             _angle) * ret_info.pt_estimation_draw.y);
    ret_info.pt_error.x = pt_estimation2.x - pt_gt2.x;
    ret_info.pt_error.y = pt_estimation2.y - pt_gt2.y;
#ifdef OUTPUT_TXT
    _fprintf(_fp, "%d, %d, %.20f,%.20f\n", ret_info.pt_estimation_level.x, 
             ret_info.pt_estimation_level.y,
            ret_info.pt_error.x,
            ret_info.pt_error.y);
    _fprintf(_fp, "%.20f,%.20f,%.20f\n", ret_info.angle, ret_info.yaw_filtered,
            ret_info.motion_state[1]);
#endif
    //for apollo ***    _timer.do_end_timer("    [-] End Delta-x Estimation, misc time: ");
    return SYS_QUIT;
}

#endif
