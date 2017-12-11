#ifndef LOCALIZATION_BASIC_STRUCT_H
#define LOCALIZATION_BASIC_STRUCT_H

#include "had_feature_define.h"
#include "had_interface.h"

#include <vector>
#include <deque>
#include <opencv2/opencv.hpp>
//#define OPENCV300

//#define USE_SLAM

#define SPEEDUP

//#define CAMERA_ONLY

//add zhuxiaohui 6.23

enum LaneMarkType {
    LANEMARK_TYPE_WHITE_DASHED = 0,  /*!< буб┴иж?Dиж?? */
    LANEMARK_TYPE_WHITE_SOLID = 1,   /*!< буб┴иж?и║ж╠?? */
    LANEMARK_TYPE_YELLOW_DASHED = 2, /*!< ??иж?Dиж?? */
    LANEMARK_TYPE_YELLOE_SOLID = 3,   /*!< ??иж?и║ж╠?? */
};

struct LANE_MARKING_HAD {
    std::vector <cv::Point2d> points_line;
    LaneMarkType lane_marking_type;
    double occupy_ratio;
    cv::Point2d start_pt;
    cv::Point2d end_pt;
    int id;
};

struct ARROWS_HAD {
    std::vector <cv::Point2d> corner_points;
    double bottom_line;
    double top_line;
};

//below old
const float FLOAT_MIN = 0.000001f;

enum SYSTEM_STATUS {
    SYS_INIT = 0,
    SYS_HADMAP,
    SYS_SCENE_PARSE,
    SYS_GROUND_DETECT,
    SYS_GROUND_MATCH_Y,
    SYS_GROUND_MATCH_X,
    SYS_VO,
    SYS_ERR,
    SYS_SUCCESS_Y,
    SYS_SUCCESS_X,
    SYS_QUIT,
};

enum HADMAP_SCENE {
    STRAIGHT = 0,
    MORE_TO_LESS = 1,
    LESS_TO_MORE = 2,
    RAMP = 3,
};

struct map_element {
    double longitude;
    double latitude;
    double height;

    std::string query_doc;
};

struct WEIGHTEDPOINT {
    cv::Point2f pt;
    float weight;
    cv::Point2d pt_d;
};

struct LANE {
    // 1. calculate from the image
    std::vector<WEIGHTEDPOINT> pts_ipm;
    cv::Vec4f vec_ipm;
    cv::Point2f start_pt_ipm;
    cv::Point2f end_pt_ipm;
    float radium_ipm;
    float theta_ipm;
    int len_ipm;

    cv::Point2f start_pt_img;
    cv::Point2f end_pt_img;
    int len_img;

    float conf;

    // 2. calculate by the logistic
    int id;
    int len_actual;
    int status;
    int type;

    float occupy_ratio;

    float dist_to_camera;
    int sub_num;

    cv::Point2f fork_pt_ipm;
    cv::Point2f fork_pt_img;
    bool is_assisted_one;

    float dist_to_vp;
};
struct  ARROW {
    std::vector<cv::Point2d> pts_ipm;
    std::vector<cv::Point2d> pts_img;
    std::string id;
    float top_line;
    float bottom_line;
};
struct LANE_COMBO {
    std::vector<LANE> lanes;
    cv::Point2f center_pt;
    HADMAP_SCENE hadmap_scene;
};

struct map_element_exjson {
    double longitude;
    double latitude;
    double height;

    std::vector<struct LANE_COMBO> l_combo;
    std::vector<struct ARROW> arrow;
};

struct CAMERAINFO {
    float fx;
    float fy;
    float cx;
    float cy;

    float pitch;
    float yaw;
    float roll;
};

struct DYNAMIC_PARAMS {
    HADMAP_SCENE hadmap_scene;

    double vo_scale;
};

struct FIXED_PARAM {
    cv::Rect roi_in;
    cv::Rect roi_out;
    CAMERAINFO camera_info;
    cv::Point2d camera_pos_img;

    std::string cnn_model;
    std::string cnn_prototxt;

    cv::Point2f camera_shift;
};

struct RETINFO {
    // 1. the final output

    // 1.1. lanes, arrows and the hadmap data
    std::vector<LANE> lanes_det;
    std::vector<LANE_COMBO> lanes_combo;
    std::vector<ARROW> arrows;
    std::vector<ARROW> arrows_det;
    LANE left_curb;
    LANE right_curb;

    // 1.2. vital parameters
    cv::Point2f vanish_point;
    float yaw;
    float yaw_filtered;
    float delta_x;
    int shift_pos;
    int channel_index;
    float angle;
    cv::Vec4f motion_state;

    cv::Point2d initial_center; //xhzhu 0808
    cv::Point2d camera_dis_2_side; //xhzhu 0815 for compare ratio
    cv::Point2d hadmap_dis_2_side;

    //for find result for big error xhzhu 0831
    double match_x;
    double short_x;
    double long_x;
    double final_x;
    double rotate_angle;

    // 1.3. the pt and corresponding error
    cv::Point2d pt_ground_truth_draw;
    cv::Point2d pt_estimation_draw;
    cv::Point2d pt_gps_draw;
    cv::Point2d pt_ground_truth;
    cv::Point2d pt_estimation_final;
    cv::Point pt_estimation_level;
    cv::Point2d pt_gps;
    cv::Point2d pt_error;
    cv::Point2d pt_error_gps;

    // 2. other output
    // 2.1. just for display
    cv::Mat road_available_img;
    cv::Mat road_available_ipm;
    cv::Mat mask;
    cv::Mat im_warp;
    cv::Mat im_ori;
    cv::Mat im_ori_warp;
    cv::Mat im_hadmap;

};

#endif
