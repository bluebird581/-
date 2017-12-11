#ifndef LOCALIZATION_LANE_INTERFACE_H
#define LOCALIZATION_LANE_INTERFACE_H

#include "basic_struct.h"

class SLLaneDetection;
class SLLaneTracking;
class LaneInterface {
public:
    LaneInterface(const float& pitch, const int& channel_num);
    ~LaneInterface();

    int do_lane_detection(cv::Mat im, int channel_num, const std::string save_name);

    LANERET get_ret();
    cv::Mat get_transformer();
    std::vector<LANE> get_lanes();
    cv::Mat get_im_warp();

private:
    SLLaneDetection* _lane_detection;
    SLLaneTracking* _lane_tracking;
    LANERET _lane_ret;
    std::vector<LANE> _lanes;
};

#endif