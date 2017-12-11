#ifndef PERCEPTION_IPM_H
#define PERCEPTION_IPM_H

#include "ground_detection.hpp"
//#include "ground_tracker.hpp"
//#include "get_hadmap_data.hpp"
//#include "ground_slam.hpp"
#include "scene_parse.hpp"
#include "hadmap_process.hpp"

#include "config.h"

class PerceptionIPM {
public:
    int do_init(const FIXED_PARAM& static_param, const std::string db_name) {
        _static_param = static_param;

        if (_hadmap_process.do_init(static_param.roi_out, db_name) == -1) {
            return -1;
        }

        int flag = _scene_parse.do_init(static_param.cnn_model, static_param.cnn_prototxt);
        _ground_detection.do_init(static_param.roi_in, static_param.roi_out,
                                  static_param.camera_pos_img,
                                  static_param.camera_shift.x);
        //_ground_tracker.do_init(static_param.roi_out, static_param.camera_shift.y);
        _ipm_transformer.do_transformation(static_param.camera_info, static_param.roi_in,
                                           static_param.roi_out);

        _ret.yaw = static_param.camera_info.yaw;
        _ret.yaw_filtered = static_param.camera_info.yaw;
        
        return flag;
    }

public:
    SYSTEM_STATUS do_detection(
        const cv::Mat& im,
        const cv::Vec3d& gps,
        DYNAMIC_PARAMS& param,
        FIXED_PARAM static_param);

    RETINFO get_ret() {
        return _ret;
    }

public:
    SYSTEM_STATUS do_preprocess(const cv::Mat& img, FIXED_PARAM& static_param);

private:
    float _former_modify_pitch;

    SYSTEM_STATUS _status;

    GroundDetection _ground_detection;
    //GroundTracker _ground_tracker;
    //GroundSlam _ground_slam;
    SceneParse _scene_parse;
    HadmapProcess _hadmap_process;
    IPMTransformer _ipm_transformer;

    cv::Mat _img_ori;
    cv::Mat _img_gray;
    cv::Mat _ipm_ori;
    cv::Mat _ipm_gray;
    cv::Mat _ipm_mask;
    std::vector<cv::Mat> _ipm_probs;

    FIXED_PARAM _static_param;

    RETINFO _ret;
    Timer _timer;
};

SYSTEM_STATUS PerceptionIPM::do_preprocess(const cv::Mat& img, FIXED_PARAM& static_param) {
#ifndef remove_clone
    _img_ori = img.clone();
#else
    _img_ori = img;
#endif
    cv::cvtColor(_img_ori, _img_gray, cv::COLOR_BGR2GRAY);
    cv::warpPerspective(_img_ori/*(cv::Rect(0, _static_param.roi_in.y,
        _img_ori.cols, _static_param.roi_in.height - _static_param.roi_in.y))*/,
                        _ipm_ori, _ipm_transformer.get_img_to_ipm(),
                        cv::Size(_static_param.roi_out.width, _static_param.roi_out.height));
    cv::cvtColor(_ipm_ori, _ipm_gray, cv::COLOR_BGR2GRAY);
#ifdef localization_display
    cv::imshow("ssss", _img_ori(cv::Rect(0, _static_param.roi_in.y,
                                         _img_ori.cols, _static_param.roi_in.height - 
                                         _static_param.roi_in.y)));
    cv::imshow("bbbb", _ipm_ori);
#endif
    return SYS_HADMAP;
}

SYSTEM_STATUS PerceptionIPM::do_detection (
    const cv::Mat& im,
    const cv::Vec3d& gps,
    DYNAMIC_PARAMS& param,
    FIXED_PARAM static_param) {
    std::cout << "[+] Begin Localization" << std::endl;
    _timer.do_start_timer();

    _status = SYS_INIT;
    _ret.pt_estimation_draw = cv::Point2d(_static_param.roi_out.width >> 1, 0);
    _ret.pt_estimation_level = cv::Point(100, 100);
    _ret.lanes_det.clear();
    _ret.lanes_combo.clear();
    _ret.arrows_det.clear();
    _ret.arrows.clear();

    //std::cout << "scale00000: " << param.vo_scale << std::endl;
    float scale_keep = param.vo_scale;

    Timer total_timer;
    Timer partial_timer;
    total_timer.do_start_timer();

    partial_timer.do_start_timer();
    _status = do_preprocess(im, static_param);
    partial_timer.do_end_timer("\t do_preprocess timer = ");

    _status = _hadmap_process.do_query_hadmap(gps, param, _ret);

    partial_timer.do_start_timer();
    _status = _scene_parse.do_scene_parse(_ipm_ori, _ipm_mask, _ipm_probs);
    partial_timer.do_end_timer("\t do_scene_parse timer = ");
   
    partial_timer.do_start_timer();
    _status = _ground_detection.do_ground_detection(_img_ori,_ipm_ori, 
                                                    _ipm_gray, _ipm_mask, 
                                                    _ipm_probs, _ipm_transformer, 
                                                    param, _ret, static_param);
    partial_timer.do_end_timer("\t do_ground_detection timer = ");
            
#ifdef use_autofit
    std::cout << "do_ground_detect: " << static_param.camera_info.roll << "\t" 
              << static_param.camera_info.pitch << "\t" << static_param.camera_info.yaw << "\t" 
              << static_param.camera_shift.x << "\t" << static_param.camera_shift.y
              << std::endl;
    _ipm_transformer.do_transformation(_static_param.camera_info, 
    _static_param.roi_in, _static_param.roi_out);
    std::cout << "adap pitch: " << _static_param.camera_info.pitch << std::endl;
    //double yaw_keep = _ret.yaw_filtered;
    //cv::imwrite("look_dier.jpg", im);
    _status = do_preprocess(im, _static_param);
    _status = _hadmap_process.do_query_hadmap(gps, param, _ret);
    _status = _scene_parse.do_scene_parse(_ipm_ori, _ipm_mask, _ipm_probs);
    _status = _ground_detection.do_ground_detection(_img_ori, _ipm_ori, _ipm_gray, 
    _ipm_mask, _ipm_probs, _ipm_transformer, param, _ret, _static_param);
#endif
    _timer.do_end_timer("ground_detect, misc time: ");
#ifdef calculate_time
    total_timer.do_end_timer("total_timer = ");
    std::cout << std::endl;
#endif
    //std::cout << "zhunagtai: " << _status << std::endl;
    return _status;
}
#endif
