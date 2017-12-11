#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include "ground_detection.hpp"
#include "ground_tracker.hpp"
#include "get_hadmap_data.hpp"
#include "ground_slam.hpp"
#include "scene_parse.hpp"
#include "hadmap_process.hpp"

#include "config.h"

std::ofstream g_pitch_out;
class MainController {
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
        _ground_tracker.do_init(static_param.roi_out, static_param.camera_shift.y);
        _ipm_transformer.do_transformation(static_param.camera_info, static_param.roi_in,
                                           static_param.roi_out);

        _ret.yaw = static_param.camera_info.yaw;
        _ret.yaw_filtered = static_param.camera_info.yaw;
        
        g_pitch_out.open("pitch.txt", std::ios::out);        

        return flag;
    }

public:
    SYSTEM_STATUS do_localization(
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
    GroundTracker _ground_tracker;
    GroundSlam _ground_slam;
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

SYSTEM_STATUS MainController::do_preprocess(const cv::Mat& img, FIXED_PARAM& static_param) {
    //0912  modify yaw
    //static_param.camera_indo.pitch += _former_modify_pitch;
    //_ipm_transformer.do_transformation(static_param.camera_info, static_param.roi_in, static_param.roi_out);

    //std::cout << img.rows << " img " << img.cols << std::endl;
#ifndef remove_clone
    _img_ori = img.clone();
#else
    _img_ori = img;
#endif
    //std::cout << _img_ori.rows << " _img_ori " << _img_ori.cols << std::endl;
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
//     _ret.im_ori_warp = _ipm_ori.clone();
    //_ret.im_ori_warp_refined = _ipm_ori.clone();
    return SYS_HADMAP;
}

SYSTEM_STATUS MainController::do_localization(
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

    std::cout << "scale00000: " << param.vo_scale << std::endl;
    float scale_keep = param.vo_scale;

#ifdef CAMERA_ONLY
    // 1. use state machine to control
    _status = do_preprocess(im, static_param);
    _status = _scene_parse.do_scene_parse(_ipm_ori,
                                          _ipm_mask, _ipm_probs);
    _status = _ground_detection.do_ground_detection(_img_ori,
              _ipm_ori, _ipm_gray, _ipm_mask, _ipm_probs, _ipm_transformer, param, _ret);

    _status = SYS_QUIT;
#else

#ifdef calculate_time 
    Timer total_timer;
    Timer partial_timer;
    total_timer.do_start_timer();
#endif
    // 1. use state machine to control
    for (int i = 0; i < 100; ++i) {
        // std::cout << "diweiduoshaoci:  " << i << std::endl;
        //std::cout << "Localization" << i << std::endl;
        switch (_status) {
        case SYS_INIT:
#ifdef calculate_time
            partial_timer.do_start_timer();
#endif
            _status = do_preprocess(im, static_param);
#ifdef calculate_time
            partial_timer.do_end_timer("\t do_preprocess timer = ");
#endif
            //             std::cout << "do_preprocess: " << static_param.camera_info.roll << "\t" << static_param.camera_info.pitch << "\t" << static_param.camera_info.yaw << "\t" << static_param.camera_shift.x << "\t" << static_param.camera_shift.y << std::endl;
            //             char nam[100];
            //             sprintf(nam, "yuan1.jpg");
            // //             cv::imwrite(nam, im);
            //
            //             std::cout << "do_preprocess: " << static_param.camera_info.cx << "\t" << static_param.camera_info.cy << "\t" << static_param.camera_info.fx << "\t" << static_param.camera_info.fy << std::endl;
            //             std::cout << "do_preprocess: " << static_param.roi_in.x << "\t" << static_param.roi_in.y << "\t" << static_param.roi_in.width << "\t" << static_param.roi_in.height << std::endl;
            //             std::cout << "do_preprocess: " << static_param.roi_out.width << "\t" << static_param.roi_out.height << "\t" << static_param.camera_pos_img.x << "\t" << static_param.camera_pos_img.y << std::endl;
            //             std::cout << "scale: " << param.vo_scale << std::endl;
            //
            //             _timer.do_end_timer("do_preprocess, misc time: ");
            break;

        case SYS_HADMAP:
#ifdef calculate_time
            partial_timer.do_start_timer();
#endif
            _status = _hadmap_process.do_query_hadmap(gps, param, _ret);
#ifdef calculate_time
            partial_timer.do_end_timer("\t do_query_hadmap timer = ");
#endif
            //                        _timer.do_end_timer("get_hadmap, misc time: ");
            break;

        case SYS_SCENE_PARSE:
#ifdef calculate_time
            partial_timer.do_start_timer();
#endif
            _status = _scene_parse.do_scene_parse(_ipm_ori,
                                                  _ipm_mask, _ipm_probs);
#ifdef calculate_time
            partial_timer.do_end_timer("\t do_scene_parse timer = ");
#endif
            //                        _timer.do_end_timer("scene_parsing, misc time: ");
            break;

        case SYS_GROUND_DETECT:
#ifdef calculate_time
            partial_timer.do_start_timer();
#endif
            _status = _ground_detection.do_ground_detection(_img_ori,
                      _ipm_ori, _ipm_gray, _ipm_mask, _ipm_probs, _ipm_transformer, param,
                    _ret, static_param);
#ifdef calculate_time
            partial_timer.do_end_timer("\t do_ground_detection timer = ");
#endif
            
#ifdef use_autofit
            std::cout << "do_ground_detect: " << static_param.camera_info.roll << "\t" << 
                static_param.camera_info.pitch << "\t" << static_param.camera_info.yaw << 
                "\t" << static_param.camera_shift.x << "\t" << static_param.camera_shift.y
                << std::endl;
            /*char nam1[100];
            sprintf(nam1, "yuan2.jpg");
            cv::imwrite(nam1, im);
            std::cout << "do_ground_detect: " << static_param.camera_info.cx << "\t" << static_param.camera_info.cy << "\t" << static_param.camera_info.fx << "\t" << static_param.camera_info.fy << std::endl;
            std::cout << "do_ground_detect: " << static_param.roi_in.x << "\t" << static_param.roi_in.y << "\t" << static_param.roi_in.width << "\t" << static_param.roi_in.height << std::endl;
            std::cout << "do_ground_detect: " << static_param.roi_out.width << "\t" << static_param.roi_out.height << "\t" << static_param.camera_pos_img.x << "\t" << static_param.camera_pos_img.y << std::endl;
            */                     // param.vo_scale = scale_keep;
            // std::cout << "scale: " << param.vo_scale << std::endl;
            _ipm_transformer.do_transformation(_static_param.camera_info, 
                _static_param.roi_in, _static_param.roi_out);
            std::cout << "adap pitch: " << _static_param.camera_info.pitch << std::endl;
            g_pitch_out << _static_param.camera_info.pitch << std::endl;
            //double yaw_keep = _ret.yaw_filtered;
            //cv::imwrite("look_dier.jpg", im);
            _status = do_preprocess(im, _static_param);
            _status = _hadmap_process.do_query_hadmap(gps, param, _ret);
            _status = _scene_parse.do_scene_parse(_ipm_ori, _ipm_mask, _ipm_probs);
            _status = _ground_detection.do_ground_detection(_img_ori, _ipm_ori, _ipm_gray, 
               _ipm_mask, _ipm_probs, _ipm_transformer, param, _ret, _static_param);
#endif
            //                      _timer.do_end_timer("ground_detect, misc time: ");
            break;

        case SYS_GROUND_MATCH_Y:
#ifdef calculate_time
            partial_timer.do_start_timer();
#endif
            _status = _ground_tracker.do_delta_y_tracker(_ipm_transformer, _ret);
            //          _timer.do_end_timer("delta_y_tracker, misc time: ");
#ifdef calculate_time
            partial_timer.do_end_timer("\t do_ground_detection timer = ");
#endif
            break;

        case SYS_VO:
#ifdef calculate_time
            partial_timer.do_start_timer();
#endif
            _status = _ground_slam.do_ground_slam(_img_gray, param.vo_scale);
            //              _timer.do_end_timer("ground_slam, misc time: ");
#ifdef calculate_time
            partial_timer.do_end_timer("\t do_ground_detection timer = ");
#endif
            break;

        case SYS_SUCCESS_Y:
#ifdef calculate_time
            partial_timer.do_start_timer();
#endif
            _status = _hadmap_process.do_delta_y_estimation(_ground_slam, _ret);
            //              _timer.do_end_timer("delta_y_estimation, misc time: ");
            break;

        case SYS_GROUND_MATCH_X:
#ifdef calculate_time
            partial_timer.do_start_timer();
#endif
            _status = _ground_tracker.do_delta_x_tracker(_ipm_transformer, _ret);

            if (_status != SYS_ERR) {
                _static_param.camera_info.yaw = _ret.yaw_filtered;
                _ipm_transformer.do_transformation(_static_param.camera_info, _static_param.roi_in,
                                                   _static_param.roi_out);
            }

            //          _timer.do_end_timer("delta_x_tracker, misc time: ");
            break;

        case SYS_SUCCESS_X:
            _status = _hadmap_process.do_delta_x_estimation(_ret);
            //          _timer.do_end_timer("delta_x_estimation, misc time: ");
            break;

        default:
            break;
        }

        //_timer.do_end_timer("Localization, misc time: ");

        if (SYS_QUIT == _status
                || SYS_ERR == _status) {
            _timer.do_end_timer("[-] End Localization, misc time: ");
            break;
        }
    }
#ifdef calculate_time
    total_timer.do_end_timer("total_timer = ");
    std::cout << std::endl;
#endif
#endif
    //fk apolo    _timer.do_end_timer("[-] End Localization, misc time: ");

    return _status;
}
#endif
