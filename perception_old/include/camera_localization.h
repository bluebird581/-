/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef LOCALIZATION_CAMERA_LOCALIZATION_H
#define LOCALIZATION_CAMERA_LOCALIZATION_H

#include <string>
#include "opencv2/opencv.hpp"

#include "basic_struct.h"
#include "config.h"

class SceneInterface;

namespace apollo {
namespace localization {

struct PosInfo {
    double longitude;
    double latitude;
    double heading_absolute; //8.2 for changcheng
    double heading_relative;
    int channel;
    //8.16 xhzhu for comparing ratio
    cv::Point2d camera_dis_2_side;
    cv::Point2d hadmap_dis_2_side;

    //for find result for big error xhzhu 0831
    double match_x;
    double short_x;
    double long_x;
    double final_x;
    double rotate_angle;

    //1127 for only perception
    std::vector<LANE> lanes_det;
    std::vector<ARROW> arrows_det;
    LANE_COMBO lanes_had;
    std::vector<ARROW> arrows_had;
};

class CameraLocalization {
public:
    CameraLocalization();
    ~CameraLocalization();

    bool init(const std::string& param_path);
    bool get_ego_position(const cv::Mat& image, const PosInfo& init_pos, PosInfo& res_pos);
private:
    SceneInterface* _scene_interface;
};

} // localization
} // apollo
#endif // MODULES_LOCALIZATION_Camera_LOCALIZATION_Camera_H_
