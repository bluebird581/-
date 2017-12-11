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

#include "camera_localization.h"
#include "scene_interface.h"

using namespace apollo::localization;

CameraLocalization::CameraLocalization() {
    _scene_interface = new SceneInterface();
}

CameraLocalization::~CameraLocalization() {
    delete _scene_interface;
}

bool CameraLocalization::init(const std::string& param_path) {
    int res = _scene_interface->do_init(param_path);

    if (res == -1) {
        return false;
    } else {
        return true;
    }
}

bool CameraLocalization::get_ego_position(const cv::Mat& image, const PosInfo& init_pos,
        PosInfo& res_pos) {
    cv::Vec3d pos(init_pos.longitude, init_pos.latitude, 0);
    int res = _scene_interface->do_scene_parsing(image, pos);

    //if (res != 0) {
    //    std::cout << "camera localization guile" << std::endl;
    //    return false;
    //}

    RETINFO ret = _scene_interface->get_ret();

    res_pos.lanes_det = ret.lanes_det;
    res_pos.arrows_det = ret.arrows_det;

    res_pos.longitude = ret.pt_estimation_final.x;
    res_pos.latitude = ret.pt_estimation_final.y;
    res_pos.heading_absolute = ret.angle * 180 / 3.1415926;//8.2 for changcheng xhzhu

    if (res_pos.heading_absolute > 180) {
        res_pos.heading_absolute -= 360;
    }

    res_pos.heading_relative = ret.yaw_filtered + 1.576;
    res_pos.channel = ret.channel_index;
    //std::cout << "Angle: " << ret.angle << "\t" << ret.yaw_filtered << std::endl;
    //xhzhu 0816 for ratio
    //if (ret.pt_estimation_level.x == 0)
    //{
    res_pos.camera_dis_2_side = ret.camera_dis_2_side;
    res_pos.hadmap_dis_2_side = ret.hadmap_dis_2_side;
    /*
    }
    else
    {
      res_pos.camera_dis_2_side = cv::Point2d(-1,-1);
      res_pos.hadmap_dis_2_side = cv::Point2d(-1,-1);
    }
    */
    //for find out which leads to big_error 0831
    res_pos.match_x = ret.match_x;
    res_pos.short_x = ret.short_x;
    res_pos.long_x = ret.long_x;
    res_pos.final_x = ret.final_x;
    res_pos.rotate_angle = ret.rotate_angle;

    res_pos.lanes_had = ret.lanes_combo[0];
    res_pos.arrows_had = ret.arrows;    

    return true;
}
