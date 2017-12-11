#include "scene_interface.h"
//#include "perception_ipm.hpp"

SceneInterface::SceneInterface() {

    //   _fixed_param.roi_in = cv::Rect(612 - 190, 370, 612 + 190, 800);
    //   _fixed_param.roi_out = cv::Rect(0, 0, 448, 224);

    //   _fixed_param.camera_info.cx = 612;
    //   _fixed_param.camera_info.cy = 512;
    //   _fixed_param.camera_info.fx = 700;
    //   _fixed_param.camera_info.fy = 700;
    //   _fixed_param.camera_info.pitch = 14.5f;
    //   _fixed_param.camera_info.yaw = 0;
    //   _fixed_param.camera_info.roll = 0;

    //   _fixed_param.camera_pos_img = cv::Point2d(612, 1024);

    //_fixed_param.cnn_model = "materials/models/1219/model_binary.caffemodel";
    //_fixed_param.cnn_prototxt = "materials/models/1219/net.prototxt";


    // 2. the detection and tracking
    _perception_ipm = new PerceptionIPM;
}

int SceneInterface::do_init(const std::string& param_path) {
    // 1. read parameters
    std::ifstream f;
    f.open(param_path.c_str(), std::ios::in);

    if (!f) {
        return -1;
    }

    f >> _fixed_param.roi_in.x >> _fixed_param.roi_in.y >> _fixed_param.roi_in.width >>
      _fixed_param.roi_in.height;
    f >> _fixed_param.roi_out.width >> _fixed_param.roi_out.height;
    f >> _fixed_param.camera_info.cx >> _fixed_param.camera_info.cy >> _fixed_param.camera_info.fx
      >> _fixed_param.camera_info.fy >> _fixed_param.camera_info.pitch >> 
      _fixed_param.camera_info.yaw >>
      _fixed_param.camera_info.roll;
    f >> _fixed_param.camera_pos_img.x >> _fixed_param.camera_pos_img.y;
    f >> _fixed_param.cnn_model;
    f >> _fixed_param.cnn_prototxt;
    f >> _fixed_param.camera_shift.x >> _fixed_param.camera_shift.y;
    std::string db_name;
    f >> db_name;

    f.close();

    // 2. check parameters

    // 3. initial the localization module
    int flag = _perception_ipm->do_init(_fixed_param, db_name);
    if (flag == -1) {
        return -1;
    }
    return flag;
}

SceneInterface::~SceneInterface() {
    delete _perception_ipm;
}

int SceneInterface::do_scene_parsing(
    const cv::Mat& im,
    const cv::Vec3d& gps) {
    return _perception_ipm->do_detection(im, gps, _dynamic_parameters, _fixed_param) 
    != SYS_QUIT;
}

RETINFO SceneInterface::get_ret() {
    return _perception_ipm->get_ret();
}
