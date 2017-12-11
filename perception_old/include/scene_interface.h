#ifndef SCENE_INTERFACE_H
#define SCENE_INTERFACE_H

#include "basic_struct.h"
//#include "perception_ipm.hpp"

class PerceptionIPM;

class SceneInterface {
public:
    SceneInterface();
    ~SceneInterface();

    int do_init(const std::string& param_path);

    int do_scene_parsing(
        const cv::Mat& im,
        const cv::Vec3d& gps);
    RETINFO get_ret();

private:
    PerceptionIPM* _perception_ipm;
    DYNAMIC_PARAMS _dynamic_parameters;
    FIXED_PARAM _fixed_param;
};

#endif
