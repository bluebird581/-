#include "lane_interface.h"
#include "lane_detection.hpp"
#include "lane_tracking.hpp"
#include "ipm.hpp"

LaneInterface::LaneInterface(const float& pitch, const int& channel_num) {
    //// 1. the basic setup for changsha
    //CAMERAINFO camera_info;
    //camera_info.cx = 1224;
    //camera_info.cy = 1024;
    //camera_info.fx = 1400;
    //camera_info.fy = 1400;
    //camera_info.pitch = 12.0f;
    //camera_info.yaw = 0;
    //camera_info.roll = 0;
    //cv::Rect roi_in = cv::Rect(1224 - 300, 850, 1224 + 300, 1450);
    //cv::Rect roi_out = cv::Rect(0, 0, 300, 200);

    // 1. the basic setup for taskid 49
    CAMERAINFO camera_info;
    //camera_info.cx = 1224;
    //camera_info.cy = 1024;
    //camera_info.fx = 1400;
    //camera_info.fy = 1400;
    //camera_info.pitch = 15.0f;
    //camera_info.yaw = 0;
    //camera_info.roll = 0;
    //cv::Rect roi_in = cv::Rect(1224 - 300, 800, 1224 + 300, 1500);
    //cv::Rect roi_out = cv::Rect(0, 0, 300, 200);

    camera_info.cx = 612;
    camera_info.cy = 512;
    camera_info.fx = 700;
    camera_info.fy = 700;
    camera_info.pitch = 14.5f;
    camera_info.yaw = 0;
    camera_info.roll = 0;
    //cv::Rect roi_in = cv::Rect(612 - 150, 400, 612 + 150, 750);
    //cv::Rect roi_out = cv::Rect(0, 0, 300, 200);
    cv::Rect roi_in = cv::Rect(612 - 370, 400, 612 + 370, 750);
    cv::Rect roi_out = cv::Rect(0, 0, 400, 200);

    cv::Point2f camera_pos_img = cv::Point2f(612, 1024);
    //cv::Point2f camera_pos_img = cv::Point2f(1024, -1024); // to be modified according to the actual pos

    // 2. the ipm
    IPMTransformer ipm_transformer;
    ipm_transformer.do_transformation(camera_info, roi_in, roi_out);
    cv::Point2f camera_pos_ipm = ipm_transformer.pt_transformation(camera_pos_img,
                                 ipm_transformer.get_img_to_ipm());

    //char path_name[1024];
    //sprintf(path_name, "trans_%d.txt", 4);

    //std::ofstream f1(path_name, std::ios::out);
    //cv::Mat trans = ipm_transformer.get_img_to_ipm();
    //for (int r = 0; r < trans.rows; ++r)
    //{
    //  for (int c = 0; c < trans.cols; ++c)
    //  {
    //      f1 << trans.at<float>(r, c) << " ";
    //  }
    //  f1 << std::endl;
    //}
    //f1.close();

    // 3. the detection and tracking
    _lane_detection = new SLLaneDetection(roi_in, roi_out, ipm_transformer, camera_pos_ipm,
                                          "models/0818/");
    _lane_tracking = new SLLaneTracking(roi_out, channel_num, ipm_transformer, camera_pos_ipm);
}

LaneInterface::~LaneInterface() {
    delete _lane_detection;
    delete _lane_tracking;
}

int LaneInterface::do_lane_detection(cv::Mat im, int channel_num, const std::string save_name) {
    _lane_detection->do_lane_detection(im, channel_num, save_name);
    _lanes = _lane_detection->get_lanes();
    _lane_tracking->do_tracking(_lanes, channel_num);
    _lane_ret = _lane_tracking->get_ret();

    return 0;
}

cv::Mat LaneInterface::get_transformer() {
    return _lane_detection->get_transformer().get_img_to_ipm();
}

LANERET LaneInterface::get_ret() {
    return _lane_ret;
}
std::vector<LANE> LaneInterface::get_lanes() {
    return _lanes;
}
cv::Mat LaneInterface::get_im_warp() {
    return _lane_detection->get_im_warp();
}
