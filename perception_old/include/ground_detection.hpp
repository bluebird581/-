#ifndef GROUND_DETECTION_H
#define GROUND_DETECTION_H

#include <string>
#include "ipm.hpp"
#include "ransac.hpp"
#include "kalman.hpp"
#include "timer.hpp"
#ifndef __GNUC__
#include <windows.h>
#undef min
#undef max
#endif

#include "config.h"
#define OPENCV300

enum NEIGHBORS {
    NORMAL = 0,
    CONFLICT = 1,
};

enum CLASS_NAME {
    CLS_BG = 0,
    CLS_ROAD = 1,
    CLS_LANE = 2,
    CLS_ARROW = 3,
    CLS_OTHER = 4,
    CLS_CAR = 5,
};

class GroundDetection {
public:
    GroundDetection() {
        _kalman_vanish_point = NULL;
    }
    ~GroundDetection() {
        if (_kalman_vanish_point) {
            delete _kalman_vanish_point;
            _kalman_vanish_point = NULL;
        }
    }

    int do_init(
        const cv::Rect& roi_in,
        const cv::Rect& roi_out,
        const cv::Point2f& camera_pos_img,
        const float& camera_shift_x) {
        // 1. the exterior parameters
        _roi_in = roi_in;
        _roi_out = roi_out;
        _vanish_point = cv::Point2f(612.0f, 350.0f);
        _camera_pos_img = camera_pos_img;
        _camera_shift_x = camera_shift_x;
        // 2. the module
        _ransac_fitting.do_init(0.9999, 0.5, 2);
        _ransac_fitting_curb.do_init(0.9999, 0.5, 3);
        // 3. the inner parameters
        _count = 0;
        return 0;
    }

    int camexpara_autofit(cv::Point3f& expara_bias, std::vector<cv::Vec4f> ego_left_lines,
                          FIXED_PARAM static_param);

    SYSTEM_STATUS do_ground_detection(
        const cv::Mat& img_ori,
        const cv::Mat& ipm_ori,
        const cv::Mat& ipm_gray,
        const cv::Mat& ipm_mask,
        const std::vector<cv::Mat>& ipm_probs,
        const IPMTransformer& ipm_transformer,
        const DYNAMIC_PARAMS& param,
        RETINFO& ret_info,
        FIXED_PARAM& static_param);

private:
    int do_lane_detection(
        RETINFO& ret_info,
        FIXED_PARAM static_param);

    int do_arrow_detection(
        RETINFO& ret_info);

    int show_image(const cv::Mat& im, const std::string win_name);

private:
    int _count;
    float _ratio_display;
    cv::Point2f _pt_show;
    float _true_pitch;
    // 1. level-2 functions
    int filter_image();
    int generate_curbs();
    int generate_driving_area();
    int generate_lane_lines_coarse();
    int generate_lane_lines_merged();
    int generate_lane_line_detections_fine(int combo_sz);

    // 2. level-3 functions
    int collect_lane_line_points(
        const cv::Mat& im,
        const cv::Mat& weights,
        const cv::Rect& roi,
        std::vector<WEIGHTEDPOINT>& pts_line);
    int find_fork_point(const std::vector<WEIGHTEDPOINT>& pts,
                        const int& mode);

    LANE init_lane_from_pts(
        std::vector<WEIGHTEDPOINT> pts_line,
        const int& mode);

    float calc_lane_lines_dist(
        const LANE& l_basic,
        const LANE& l_test);
    int merge_two_lanes(LANE& l1, const LANE& l2);

    //std::vector<std::vector<NEIGHBORS> > fill_weights();
    //int find_best_combo(
    //  std::vector<LANE> &combo_best,
    //  std::vector<LANE> &combo_tmp,
    //  const std::vector<std::vector<NEIGHBORS> > &weights,
    //  float &score_max,
    //  int level,
    //  int statue,
    //  const int &sz,
    //  const int &sz_thres);
    float calc_vp_to_vec(
        const cv::Point2f& start_pt,
        const cv::Point2f& end_pt);
    int calc_lane_line_intersection(
        const cv::Point2f& start_pt1,
        const cv::Point2f& end_pt1,
        const cv::Point2f& start_pt2,
        const cv::Point2f& end_pt2,
        cv::Point2f& insection_point);

private:
    // 1. the functional module
    IPMTransformer _ipm_transformer;
    RansacFitting _ransac_fitting;
    RansacFitting _ransac_fitting_curb;
    Kalman* _kalman_vanish_point;
    Timer _timer;

    cv::Mat _ipm_warp_mask;

    cv::Mat _img_ori;
    cv::Mat _ipm_ori;
    cv::Mat _ipm_filter;
    cv::Mat _mask;
    cv::Mat _ipm_mask;
    cv::Mat _ipm_gray;
    cv::Mat _ipm_driving_area;
    cv::Mat _reference;
    std::vector<cv::Mat> _probs;
    std::vector<cv::Mat> _ipm_probs;

    cv::Mat _hist_ipm;
    cv::Mat _hist_ipm_all;

    // 3. the key parameters
    cv::Rect _roi_in;
    cv::Rect _roi_out;
    cv::Point2f _camera_pos_ipm;
    cv::Point2f _camera_pos_img;

    std::vector<LANE> _lanes;
    std::vector<ARROW> _arrows;

    LANE _left_curb;
    LANE _right_curb;
    //std::vector<WEIGHTEDPOINT> _left_curb;
    //std::vector<WEIGHTEDPOINT> _right_curb;
    //cv::Vec4f _left_vec;
    //cv::Vec4f _right_vec;

    cv::Point2f _vanish_point;
    float _camera_shift_x;

    // 4. others
    HADMAP_SCENE _scene;
};

int GroundDetection::filter_image() {
    // 1. blur the image
    cv::Mat ipm_smooth;
    cv::blur(_ipm_gray, ipm_smooth, cv::Size(3, 9));
    // 2. middle * 2 - left - right if middle > left and middle > right
    _ipm_filter = cv::Mat::zeros(_roi_out.height, _roi_out.width, CV_32F);
    const int x_shift = 3;
    const int x_left_shift = x_shift + (x_shift >> 1);

    for (int i = 0; i < _roi_out.height; ++i) {
        uchar* p1 = ipm_smooth.ptr<uchar>(i);
        float* p2 = _ipm_filter.ptr<float>(i);

        // 2.1. scan each row
        for (int j = x_left_shift; j < _roi_out.width - x_left_shift; ++j) {
            int middle = p1[j];
            int left = p1[j - x_shift];
            int right = p1[j + x_shift];

            if (middle > left && middle > right) {
                p2[j] = (middle << 1) - left - right;
            }
        }
    }

    // 3. normalization
    cv::normalize(_ipm_filter, _ipm_filter, 1.0f, 0.0f, CV_MINMAX);
#ifdef localization_display
    cv::imshow("_ipm_filter", _ipm_filter);
#endif
    return 0;
}

int GroundDetection::generate_curbs() {
    // 1. find the left & right curb
    cv::Mat im_pure_road = _ipm_mask == CLS_BG;
    cv::Mat im_car = _ipm_mask == CLS_CAR;
    cv::Mat im_draw;
    cv::cvtColor(im_pure_road, im_draw, cv::COLOR_GRAY2BGR);
    int interval = 20;

    for (int r = 0; r < _roi_out.height; ++r) {
        uchar* p_cur = im_pure_road.ptr<uchar>(r) + interval;
        uchar* p_right = p_cur + 1;
        uchar* p_car_right = im_car.ptr<uchar>(r) + interval + 1;
        float* p2 = _hist_ipm_all.ptr<float>(0) + interval;

        for (int c = interval; c < _roi_out.width >> 1; ++c) {
            // 1.1. if next is not road and not car, the cur is the road
            if (*p_right == 0) {
                // 1.1.1. if next is car, skip it
                if (*p_car_right == 255) {
                    continue;
                }

                if (*p_cur == 255 && r < *p2 - 5) {
                    //cv::circle(im_draw, cv::Point(c, r), 3, cv::Scalar(0, 0, 255), -1);
                    WEIGHTEDPOINT pt;
                    pt.pt = cv::Point(r, c);
                    pt.weight = 1;
                    _left_curb.pts_ipm.push_back(pt);
                }

                break;
            }

            ++p_right;
            ++p_car_right;
            ++p_cur;
            ++p2;
        }
    }

    for (int r = 0; r < _roi_out.height; ++r) {
        uchar* p_cur = im_pure_road.ptr<uchar>(r) + _roi_out.width - interval - 1;
        uchar* p_left = p_cur - 1;
        uchar* p_car_left = im_car.ptr<uchar>(r) + _roi_out.width - interval - 2;
        float* p2 = _hist_ipm_all.ptr<float>(0) + _roi_out.width - interval - 1;

        for (int c = _roi_out.width - 1 - interval; c > _roi_out.width >> 1; --c) {
            if (*p_left == 0) {
                if (*p_car_left == 255) {
                    continue;
                }

                if (*p_cur == 255 && r < *p2 - 5) {
                    //cv::circle(im_draw, cv::Point(c, r), 3, cv::Scalar(255, 0, 0), -1);
                    WEIGHTEDPOINT pt;
                    pt.pt = cv::Point(r, c);
                    pt.weight = 1;
                    _right_curb.pts_ipm.push_back(pt);
                }

                break;
            }

            --p_left;
            --p_car_left;
            --p_cur;
            --p2;
        }
    }

    if (_left_curb.pts_ipm.size() > _roi_out.height * 2.0 / 3) {
        _left_curb.pts_ipm = _ransac_fitting_curb.ransac_fit_poly_weighted(_left_curb.pts_ipm,
                             _left_curb.vec_ipm);

        if (_left_curb.pts_ipm.size() > _roi_out.height * 0.5) {
            _left_curb.start_pt_ipm = _left_curb.pts_ipm[0].pt;
            _left_curb.end_pt_ipm = _left_curb.pts_ipm[_left_curb.pts_ipm.size() - 1].pt;
            _left_curb.dist_to_camera = _camera_pos_ipm.x + _camera_shift_x -
                                        (_left_curb.vec_ipm[0] * _roi_in.height * _roi_in.height
                                         + _left_curb.vec_ipm[1] * _roi_in.height +
                                         _left_curb.vec_ipm[2]);
            //for (int r = 0; r < _roi_out.height; ++r)
            //{
            //  int c = _left_vec[0] * r * r + _left_vec[1] * r + _left_vec[2];
            //  cv::circle(im_draw, cv::Point(c, r), 1, cv::Scalar(0, 255, 0), -1);
            //}
            _left_curb.start_pt_img = _ipm_transformer.pt_transformation(_left_curb.start_pt_ipm,
                                      _ipm_transformer.get_ipm_to_img());
            _left_curb.end_pt_img = _ipm_transformer.pt_transformation(_left_curb.end_pt_ipm,
                                    _ipm_transformer.get_ipm_to_img());
            //_left_curb.start_pt_img.y += _roi_in.y;
            //_left_curb.end_pt_img.y += _roi_in.y;
        } else {
            _left_curb.pts_ipm.clear();
        }
    } else {
        _left_curb.pts_ipm.clear();
    }

    if (_right_curb.pts_ipm.size() > _roi_out.height * 2.0 / 3) {
        _right_curb.pts_ipm = _ransac_fitting_curb.ransac_fit_poly_weighted(_right_curb.pts_ipm,
                              _right_curb.vec_ipm);

        if (_right_curb.pts_ipm.size() > _roi_out.height * 0.5) {
            _right_curb.start_pt_ipm = _right_curb.pts_ipm[0].pt;
            _right_curb.end_pt_ipm = _right_curb.pts_ipm[_right_curb.pts_ipm.size() - 1].pt;
            _right_curb.dist_to_camera = -(_camera_pos_ipm.x + _camera_shift_x) +
                                         (_right_curb.vec_ipm[0] * _roi_in.height * _roi_in.height
                                          + _right_curb.vec_ipm[1] * _roi_in.height
                                          + _right_curb.vec_ipm[2]);
            //for (int r = 0; r < _roi_out.height; ++r)
            //{
            //  int c = _right_vec[0] * r * r + _right_vec[1] * r + _right_vec[2];
            //  cv::circle(im_draw, cv::Point(c, r), 1, cv::Scalar(0, 255, 0), -1);
            //}
            _right_curb.start_pt_img = _ipm_transformer.pt_transformation(_right_curb.start_pt_ipm,
                                       _ipm_transformer.get_ipm_to_img());
            _right_curb.end_pt_img = _ipm_transformer.pt_transformation(_right_curb.end_pt_ipm,
                                     _ipm_transformer.get_ipm_to_img());
            //_right_curb.start_pt_img.y += _roi_in.y;
            //_right_curb.end_pt_img.y += _roi_in.y;
        } else {
            _right_curb.pts_ipm.clear();
        }
    } else {
        _right_curb.pts_ipm.clear();
    }

#ifdef localization_display

    for (int i = 0; i < _left_curb.pts_ipm.size(); ++i) {
        cv::circle(im_draw, _left_curb.pts_ipm[i].pt, 3, cv::Scalar(255, 0, 0), -1);
    }

    for (int i = 0; i < _right_curb.pts_ipm.size(); ++i) {
        cv::circle(im_draw, _right_curb.pts_ipm[i].pt, 3, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("curb", im_draw);
    //  cv::waitKey(0);
#endif
    return 0;
}

int GroundDetection::generate_driving_area() {
    // 1. driving area = road + lane + arrow + other
    cv::Mat driving_area = ((_ipm_mask == CLS_ROAD)
                            | (_ipm_mask == CLS_LANE)
                            | (_ipm_mask == CLS_ARROW)
                            | (_ipm_mask == CLS_OTHER));
    driving_area.copyTo(_ipm_driving_area);
    // 2. find the maximal region as the actual road
    int max_idx = -1;
    int max_num = 0;
    std::vector<std::vector<cv::Point> > contours;
    findContours(driving_area, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    for (int i = 0; i < contours.size(); ++i) {
        cv::Rect rt = cv::boundingRect(contours[i]);
        int sz = cv::contourArea(contours[i]);

        // 2.1. the region should be in the center
        if (max_num < sz && rt.x < _camera_pos_ipm.x - 10
                && rt.x + rt.width > _camera_pos_ipm.y + 10) {
            max_num = sz;
            max_idx = i;
        }
    }

    if (max_idx != -1) {
        _ipm_driving_area = cv::Mat::zeros(_ipm_mask.rows, _ipm_mask.cols, CV_8U);
        cv::drawContours(_ipm_driving_area, contours, max_idx, cv::Scalar(255), -1);
    }

#ifdef localization_display
    cv::imshow("_ipm_driving_area", _ipm_driving_area);
#endif
    return 0;
}

int GroundDetection::collect_lane_line_points(
    const cv::Mat& ipm_mask,
    const cv::Mat& ipm_weights,
    const cv::Rect& roi,
    std::vector<WEIGHTEDPOINT>& pts_line) {
    // 1. scan each row, preserve only a pixel for one region
    for (int i = roi.y; i < roi.y + roi.height; ++i) {
        const uchar* p1 = ipm_mask.ptr<uchar>(i);
        const float* p2 = ipm_weights.ptr<float>(i);
        // 1.1. calculate the avg. weight and pos
        float pos_w_sum = 0.0f;
        float w_sum = 0.0f;
        int w_count = 0;

        for (int j = roi.x; j < roi.x + roi.width; ++j) {
            if (p1[j] == 128) {
                pos_w_sum += j * p2[j];
                w_sum += p2[j];
                ++w_count;
            }
        }

        if (w_count) {
            WEIGHTEDPOINT pt;
            pt.pt = cv::Point2f(i, pos_w_sum / w_sum);
            pt.weight = w_sum / w_count;
            pts_line.push_back(pt);
        }
    }

    return 0;
}

int GroundDetection::find_fork_point(
    const std::vector<WEIGHTEDPOINT>& pts,
    const int& mode) {
    // 1. use the first and last point as the corresponding
    int sz = pts.size();
    cv::Point2f pt_start = pts[0].pt;
    cv::Point2f pt_end = pts[sz - 1].pt;
    cv::Point2f vec_basis = cv::Point2f(pt_start.x - pt_end.x, pt_start.y - pt_end.y);
    float vec_basis_norm = sqrt(vec_basis.x * vec_basis.x + vec_basis.y * vec_basis.y);
    vec_basis.x /= vec_basis_norm;
    vec_basis.y /= vec_basis_norm;
    // 2. from the end point, collect the appropriate points
    float old_val = 0.0f;
    int dir_line = 0;
    float maximal_dist = INT_MIN;
    int maximal_idx = -1;

    if (mode == 0) {
        for (int i = 1; i < sz - 1; ++i) {
            cv::Point2f vec_test = cv::Point2f(pts[i].pt.x - pt_end.x, pts[i].pt.y - pt_end.y);
            float new_val = vec_basis.x * vec_test.y - vec_basis.y * vec_test.x;

            if (!dir_line) {
                dir_line = new_val > 0 ? 1 : -1;
            } else {
                if (new_val * dir_line < 0) {
                    break;
                }
            }

            float tmp_dist = fabs(new_val);

            if (tmp_dist > maximal_dist) {
                maximal_dist = tmp_dist;
                maximal_idx = i;
            }
        }
    } else {
        for (int i = sz - 2; i >= 0; --i) {
            cv::Point2f vec_test = cv::Point2f(pts[i].pt.x - pt_end.x, pts[i].pt.y - pt_end.y);
            float new_val = vec_basis.x * vec_test.y - vec_basis.y * vec_test.x;

            if (!dir_line) {
                dir_line = new_val > 0 ? 1 : -1;
            } else {
                if (new_val * dir_line < 0) {
                    break;
                }
            }

            float tmp_dist = fabs(new_val);

            if (tmp_dist > maximal_dist) {
                maximal_dist = tmp_dist;
                maximal_idx = i;
            }
        }
    }

    // 3. if fork or curve, return the fork point; otherwise neglect the fork point
    if (maximal_dist < 5) {
        return -1;
    } else {
        return maximal_idx;
    }
}

LANE GroundDetection::init_lane_from_pts(
    std::vector<WEIGHTEDPOINT> pts_line,
    const int& mode) {
    LANE lane;
    lane.fork_pt_img = cv::Point2f(-100.0f, -100.0f);
    lane.fork_pt_ipm = cv::Point2f(-100.0f, -100.0f);
    lane.is_assisted_one = false;
    int max_sz = std::min(56, int(pts_line.size()));// mode ? _roi_out.height : 60;
    // 1. according to the scene, choose the best line fitting mode
    cv::Vec4f vec_polyfit_1;
    //cv::Vec4f vec_polyfit_3;
    std::vector<WEIGHTEDPOINT> preserved_pts;
    //if (STRAIGHT == _scene)
    //{
    // 1.1. if no fork, only fit vec_polyfit_1 using all points
    pts_line.erase(pts_line.begin(), pts_line.end() - max_sz);
    preserved_pts = _ransac_fitting.ransac_fitting_weighted(pts_line, vec_polyfit_1);
    //}
    //else if (MORE_TO_LESS == _scene)
    //{
    //  // 1.2. if fork, find the fork point
    //  int idx = find_fork_point(pts_line, 0);

    //  if (idx != -1 && !mode)
    //  {
    //      int erase_sz = std::min(max_sz, idx);
    //      lane.fork_pt_ipm = cv::Point2f(pts_line[idx].pt.y, pts_line[idx].pt.x);
    //      lane.fork_pt_img = _ipm_transformer.pt_transformation(lane.fork_pt_ipm,
    //          _ipm_transformer.get_ipm_to_img());
    //      pts_line.erase(pts_line.begin() + erase_sz, pts_line.end());
    //  }
    //  else
    //  {
    //      int erase_sz = std::min(max_sz, int(pts_line.size()));
    //      pts_line.erase(pts_line.begin() + erase_sz, pts_line.end());
    //  }
    //  preserved_pts = _ransac_fitting.ransac_fitting_weighted(pts_line, vec_polyfit_1);

    //}
    //else if (LESS_TO_MORE == _scene)
    //{
    //  // 1.2. if fork, find the fork point
    //  int idx = find_fork_point(pts_line, 1);

    //  if (idx != -1 && !mode)
    //  {
    //      int erase_sz = std::min(max_sz, int(pts_line.size()) - idx);
    //      lane.fork_pt_ipm = cv::Point2f(pts_line[idx].pt.y, pts_line[idx].pt.x);
    //      lane.fork_pt_img = _ipm_transformer.pt_transformation(lane.fork_pt_ipm,
    //          _ipm_transformer.get_ipm_to_img());
    //      pts_line.erase(pts_line.begin(), pts_line.end() - erase_sz);
    //  }
    //  else
    //  {
    //      int erase_sz = std::min(max_sz, int(pts_line.size()));
    //      pts_line.erase(pts_line.begin(), pts_line.end() - erase_sz);
    //  }
    //  preserved_pts = _ransac_fitting.ransac_fitting_weighted(pts_line, vec_polyfit_1);

    //}
    //else if (3 == _scene)
    //{
    //}
    //else
    //{
    //}

    if (preserved_pts.size() >= 4) {
        // 1.1. prepare the ipm data
        lane.vec_ipm = vec_polyfit_1;
        lane.pts_ipm = preserved_pts;
        lane.start_pt_ipm = lane.pts_ipm[0].pt;
        lane.end_pt_ipm = lane.pts_ipm[lane.pts_ipm.size() - 1].pt;
        lane.len_ipm = sqrt((lane.end_pt_ipm.x - lane.start_pt_ipm.x) *
                            (lane.end_pt_ipm.x - lane.start_pt_ipm.x) +
                            (lane.end_pt_ipm.y - lane.start_pt_ipm.y) *
                            (lane.end_pt_ipm.y - lane.start_pt_ipm.y));
        _ipm_transformer.xy_to_rtheta(lane.start_pt_ipm, lane.end_pt_ipm, lane.radium_ipm,
                                      lane.theta_ipm, cv::Point2f(0, _ipm_ori.rows / 2));
        // 1.2. prepare the img data
        //lane.start_pt_img = _ipm_transformer.pt_transformation(lane.start_pt_ipm, _ipm_transformer.get_ipm_to_img());
        //lane.end_pt_img = _ipm_transformer.pt_transformation(lane.end_pt_ipm, _ipm_transformer.get_ipm_to_img());
        //lane.len_img = sqrt((lane.end_pt_img.x - lane.start_pt_img.x) *
        //  (lane.end_pt_img.x - lane.start_pt_img.x) +
        //  (lane.end_pt_img.y - lane.start_pt_img.y) *
        //  (lane.end_pt_img.y - lane.start_pt_img.y));
        // 1.3. calculate the prob
        float prob = 0.0f;
        int count = 0;
        int sz = preserved_pts.size();

        for (int i = 0; i < sz; ++i) {
            prob += preserved_pts[i].weight;
            ++count;
        }

        lane.conf = prob / count / 255;
    }

    return lane;
}

int GroundDetection::generate_lane_lines_coarse() {
    // 1. combine the filter and cnn mask
    cv::Mat ipm_bw1;
    cv::threshold(_ipm_filter, ipm_bw1, 0.05, 1.0, CV_THRESH_BINARY);
    ipm_bw1.convertTo(ipm_bw1, CV_8U);
    //cv::imshow("sbb", ipm_bw1 * 255);
    cv::Mat ipm_bw2 = (_ipm_mask == CLS_LANE) & _ipm_driving_area;
    cv::Mat ipm_bw = ipm_bw1 & ipm_bw2;
    //cv::imshow("ipm_bw1", ipm_bw1 * 255);
    //cv::imshow("ipm_bw2", ipm_bw2);
    //cv::imshow("ipm_bw", ipm_bw * 255);
    // 2. find all contours, get the proposals
    cv::Mat ipm_bw_tmp = ipm_bw.clone();
    std::vector<std::vector<cv::Point> > contours;
    findContours(ipm_bw_tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cv::Mat ipm_weight;
    _ipm_probs[CLS_LANE].convertTo(ipm_weight, CV_32FC1);
    //cv::imshow("sb", ipm_weight / 255);
    ipm_weight = _ipm_filter.mul(ipm_weight);
    int contours_sz = contours.size();

    for (int i = 0; i < contours_sz; ++i) {
        // 2.1. too small contour should be deleted
        if (contours[i].size() < 20) {
            continue;
        }

        // 2.2. too short line should be deleted
        cv::RotatedRect rrt = cv::minAreaRect(contours[i]);
        int w = std::max(rrt.size.width, rrt.size.height);
        int h = std::min(rrt.size.width, rrt.size.height);
        float angle = rrt.angle;

        if (rrt.size.width < rrt.size.height) {
            angle -= 90;
        }

        if (w < 12
                || angle < -135
                || angle > -45) {
            continue;
        }

        // 2.3. lane line fitting
        ipm_bw.copyTo(ipm_bw_tmp);
        cv::drawContours(ipm_bw_tmp, contours, i, cv::Scalar(128), CV_FILLED);
        cv::Rect rt = boundingRect(contours[i]);
        std::vector<WEIGHTEDPOINT> pts_line;
        collect_lane_line_points(ipm_bw_tmp, ipm_weight, rt, pts_line);
        // 2.4. a new lane line proposal
        LANE lane = init_lane_from_pts(pts_line, 0);

        if (lane.len_ipm < 12 || lane.conf < 0.2) {
            continue;
        }

        lane.id = i;
        lane.len_actual = rt.height;
        lane.status = 0;
        lane.dist_to_camera = 0;
        lane.sub_num = 1;
        float pixel_sum = _hist_ipm.at<float>(0, int(lane.end_pt_ipm.x + lane.start_pt_ipm.x) / 2);
        lane.occupy_ratio = lane.len_actual / pixel_sum;

        if (lane.len_actual / pixel_sum > 0.9) {
            lane.type = 1;
        } else {
            lane.type = 0;
        }

        _lanes.push_back(lane);
    }

    return 0;
}

//int GroundDetection::generate_lanes_candidate()
//{
//  // 1. threshold and get contours
//
//  cv::Mat im_bw2;
//  cv::threshold(_ipm_filter, im_bw2, 0.15, 1, CV_THRESH_BINARY);
//  im_bw2.convertTo(im_bw2, CV_8UC1, 1);
//
//  cv::Mat im_bw = ((_ipm_mask == CLS_LANE) \
//      & im_bw2 \
//      & _ipm_road);
//
//  cv::imshow("filter", _ipm_filter);
//  cv::imshow("filter_all", im_bw * 255);
//
//  //cv::Mat im_weight;
//  //im_bw.convertTo(im_weight, CV_32FC1);
//
//  cv::Mat im_bw_tmp = im_bw.clone();
//  std::vector<std::vector<cv::Point> > contours;
//  findContours(im_bw_tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//
//  cv::Mat prob;
//  _ipm_probs[CLS_LANE].convertTo(prob, CV_32FC1);
//  _reference = _ipm_filter.mul(prob);
//
//  // 2. scan each contour, generate the candidate lanes
//  for (int i = 0; i < contours.size(); ++i)
//  {
//      // 2.0. if the points is too small, kick out
//      if (contours[i].size() < 20)
//      {
//          continue;
//      }
//
//      // 2.1. filter by angle and length
//      cv::RotatedRect rrt = cv::minAreaRect(contours[i]);
//
//      int w = std::max(rrt.size.width, rrt.size.height);
//      int h = std::min(rrt.size.width, rrt.size.height);
//      float angle = rrt.angle;
//
//      if (rrt.size.width < rrt.size.height)
//      {
//          angle -= 90;
//      }
//
//      if (w < 12
//          || angle < -135
//          || angle > -45)
//      {
//          continue;
//      }
//
//      // 2.2. ransac fitting
//      im_bw.copyTo(im_bw_tmp);
//      drawContours(im_bw_tmp, contours, i, cv::Scalar(128), CV_FILLED);
//      cv::Rect rt = boundingRect(contours[i]);
//
//      std::vector<WEIGHTEDPOINT> pts_line;
//
//      collect_points(im_bw_tmp, _reference, rt, pts_line);
//
//      //cv::imshow("prob", prob / 255);
//      //cv::imshow("_im_filter", _im_filter);
//      //cv::imshow("filter", _im_filter.mul(prob) / 255);
//
//      LANE lane = init_lane_from_pts(pts_line);
//      if (lane.len_ipm < 12)
//      {
//          continue;
//      }
//
//      lane.id = i;
//      lane.len_actual = rt.height;
//      lane.status = 0;
//      lane.dist_to_camera = 0;
//      lane.sub_num = 1;
//
//      float pixel_sum = _hist_ipm.at<float>(0, lane.end_pt_ipm.y);
//      if (lane.len_actual / pixel_sum > 0.9)
//      {
//          lane.type = 1;
//      }
//      else
//      {
//          lane.type = 0;
//      }
//
//      _lanes.push_back(lane);
//  }
//
//  return 0;
//}

bool sort_by_conf_desc(const LANE& l1, const LANE& l2) {
    return l1.conf > l2.conf;
}

bool sort_by_start_y_desc(const LANE& l1, const LANE& l2) {
    return l1.start_pt_ipm.y > l2.start_pt_ipm.y;
}

bool sort_by_y_asc(const WEIGHTEDPOINT& pt1, const WEIGHTEDPOINT& pt2) {
    return pt1.pt.y < pt2.pt.y;
}

int GroundDetection::merge_two_lanes(LANE& l1, const LANE& l2) {
    // 1. construct the pts
    LANE lane;
    lane.pts_ipm.clear();
    std::vector<WEIGHTEDPOINT> pts;
    pts.insert(pts.end(), l2.pts_ipm.begin(), l2.pts_ipm.end());
    pts.insert(pts.end(), l1.pts_ipm.begin(), l1.pts_ipm.end());

    // 2. use the weighted pts for line fitting
    for (int i = 0; i < l2.pts_ipm.size(); ++i) {
        WEIGHTEDPOINT pt = l2.pts_ipm[i];
        pt.weight *= pt.pt.y / _roi_out.height;
        std::swap(pt.pt.x, pt.pt.y);
        lane.pts_ipm.push_back(pt);
    }

    for (int i = 0; i < l1.pts_ipm.size(); ++i) {
        WEIGHTEDPOINT pt = l1.pts_ipm[i];
        pt.weight *= pt.pt.y / _roi_out.height;
        std::swap(pt.pt.x, pt.pt.y);
        lane.pts_ipm.push_back(pt);
    }

    lane = init_lane_from_pts(lane.pts_ipm, 1);
    lane.pts_ipm = pts;

    // 3. the global parameters
    if (l1.fork_pt_ipm.x >= 0.0f && l1.fork_pt_ipm.y >= 0.0f) {
        lane.fork_pt_ipm = l1.fork_pt_ipm;
        lane.fork_pt_img = l1.fork_pt_img;
    } else if (l2.fork_pt_ipm.x >= 0.0f && l2.fork_pt_ipm.y >= 0.0f) {
        lane.fork_pt_ipm = l2.fork_pt_ipm;
        lane.fork_pt_img = l2.fork_pt_img;
    }

    lane.conf = l1.conf + l2.conf;
    lane.type = 0;
    lane.sub_num = l1.sub_num + l2.sub_num;
    lane.len_actual = l1.len_actual + l2.len_actual;
    l1 = lane;
    return 0;
}

float GroundDetection::calc_lane_lines_dist(
    const LANE& l_basic,
    const LANE& l_test) {
    float dist_avg = 0.0f;
    int sz = l_test.pts_ipm.size();

    for (int i = 0; i < sz; ++i) {
        dist_avg += _ransac_fitting.get_dist(l_basic.vec_ipm, cv::Point2f(l_test.pts_ipm[i].pt.y,
                                             l_test.pts_ipm[i].pt.x));
    }

    return dist_avg / sz;
}

float GroundDetection::calc_vp_to_vec(
    const cv::Point2f& start_pt,
    const cv::Point2f& end_pt) {
    // 1. calculate the normal vector
    cv::Point2f vec = end_pt - start_pt;
    float norm_val = sqrt(vec.x * vec.x + vec.y * vec.y);
    vec = cv::Point2f(-vec.y / norm_val, vec.x / norm_val);
    // 2. calculate the distance
    cv::Point2f vec_diff = _vanish_point - start_pt;
    return fabs(vec.x * vec_diff.x + vec.y * vec_diff.y);
}

int GroundDetection::generate_lane_lines_merged() {
    // 1. sort lane lines by the start y desc
    std::sort(_lanes.begin(), _lanes.end(), sort_by_start_y_desc);

    // 2. scan each lane line pair
    for (int i = 0; i < _lanes.size(); ++i) {
        for (int j = i + 1; j < _lanes.size(); ++j) {
            // 2.1. skip the overlap
            cv::Point2f start_pt = _lanes[i].start_pt_ipm.y > _lanes[j].start_pt_ipm.y ?
                                   _lanes[i].start_pt_ipm : _lanes[j].start_pt_ipm;
            cv::Point2f end_pt = _lanes[i].end_pt_ipm.y < _lanes[j].end_pt_ipm.y ?
                                 _lanes[i].end_pt_ipm : _lanes[j].end_pt_ipm;

            if (start_pt.y < end_pt.y) {
                continue;
            }

            // 2.2. if the distance is less than 20, merge it
            float dist1 = calc_lane_lines_dist(_lanes[i], _lanes[j]);
            float dist2 = calc_lane_lines_dist(_lanes[j], _lanes[i]);

            if (dist1 < 20 && dist2 < 20) {
                merge_two_lanes(_lanes[i], _lanes[j]);
                _lanes.erase(_lanes.begin() + j);
                --j;
            }
        }
    }

    // 3. re-arrange the lane lines

    if (_kalman_vanish_point) {
        std::vector<float> measure_vals;
        measure_vals = _kalman_vanish_point->do_predict();
        _vanish_point.x = measure_vals[0];
        _vanish_point.y = measure_vals[1];
    } else {
        _kalman_vanish_point = new Kalman;
        std::vector<float> measure_vals;
        measure_vals.push_back(_vanish_point.x);
        measure_vals.push_back(_vanish_point.y);
        _kalman_vanish_point->init(2, 2, measure_vals, 1e-5, 1e-1);
    }

    for (int i = 0; i < _lanes.size(); ++i) {
        // 3.1. if too short, or too far, remove it
        if (_lanes[i].len_actual < (_roi_out.height >> 3)
                || _lanes[i].end_pt_ipm.y < (_roi_out.height >> 1)) {
            _lanes.erase(_lanes.begin() + i);
            --i;
            continue;
        }

        // 3.2. if violate the vanish point, remove it
        float dist = calc_vp_to_vec(_lanes[i].start_pt_img, _lanes[i].end_pt_img);
        //if (dist > 80)
        //repair by liuchao27
        //if (dist > 100)
        /*if (dist > 100)
        {
            _lanes.erase(_lanes.begin() + i);
            --i;
            continue;
        }*/
        _lanes[i].dist_to_vp = dist;
        _lanes[i].conf /= _lanes[i].sub_num;
        float pixel_sum = _hist_ipm.at<float>(0,
        int(_lanes[i].end_pt_ipm.x + _lanes[i].start_pt_ipm.x) >> 1);
        _lanes[i].occupy_ratio = _lanes[i].len_actual / pixel_sum;

        if (_lanes[i].len_actual / pixel_sum > 0.9
                && _lanes[i].sub_num == 1) {
            _lanes[i].type = 1;
        } else {
            _lanes[i].type = 0;
        }
    }

    return 0;
}

bool sort_by_radium_asc(const LANE& l1, const LANE& l2) {
    return l1.radium_ipm < l2.radium_ipm;
}

bool sort_by_dist_asc(const LANE& l1, const LANE& l2) {
    return l1.dist_to_camera < l2.dist_to_camera;
}
//
//std::vector<std::vector<NEIGHBORS> > GroundDetection::fill_weights()
//{
//  int sz = _lanes.size();
//
//  // 1. make space
//  std::vector<std::vector<NEIGHBORS> > weights(sz);
//  for (int i = 0; i < sz; ++i)
//  {
//      weights[i].resize(sz);
//  }
//
//  // 2. scan lane pair
//  for (int i = 0; i < sz; ++i)
//  {
//      for (int j = i + 1; j < sz; ++j)
//      {
//          if (fabs(_lanes[i].theta_ipm - _lanes[j].theta_ipm) < 5 * CV_PI / 180
//              && fabs(_lanes[i].radium_ipm - _lanes[j].radium_ipm) < 20)
//          {
//              weights[i][j] = weights[j][i] = CONFLICT;
//          }
//          else
//          {
//              weights[i][j] = weights[j][i] = NORMAL;
//          }
//
//          continue;
//
//          // 2.1. calculate the intersection pt
//          cv::Point2f intersection_pt;
//          int count = 0;
//          calc_lane_line_intersection(_lanes[i].start_pt_ipm, _lanes[i].end_pt_ipm,
//              _lanes[j].start_pt_ipm, _lanes[j].end_pt_ipm, intersection_pt);
//
//          //// 2.2. if the pt is in the range of the image, the two lane is conflict
//          //if (0 <= intersection_pt.x && intersection_pt.x < _roi_out.width
//          //  && 0 <= intersection_pt.y && intersection_pt.y < _roi_out.height)
//          //{
//          //  weights[i][j] = weights[j][i] = CONFLICT;
//          //}
//          //else
//          //{
//          //  weights[i][j] = weights[j][i] = NORMAL;
//          //}
//
//          float dist1 = calc_lane_lines_dist(_lanes[i], _lanes[j]);
//          float dist2 = calc_lane_lines_dist(_lanes[j], _lanes[i]);
//
//          // 2.1. if the angle is bigger, or the radium smaller
//          if (dist1 < 60 || dist2 < 60)
//          {
//              weights[i][j] = weights[j][i] = CONFLICT;
//          }
//          else
//          {
//              weights[i][j] = weights[j][i] = NORMAL;
//          }
//      }
//  }
//
//  return weights;
//}
//
//int GroundDetection::find_best_combo(
//  std::vector<LANE> &combo_best,
//  std::vector<LANE> &combo_tmp,
//  const std::vector<std::vector<NEIGHBORS> > &weights,
//  float &score_max,
//  int level,
//  int statue,
//  const int &sz,
//  const int &sz_thres)
//{
//  // 1. find a better combo
//  if (combo_tmp.size() > 0)
//  {
//      float score_tmp = 0.0f;
//
//      for (int i = 0; i < combo_tmp.size(); ++i)
//      {
//          score_tmp += combo_tmp[i].conf + (combo_tmp[i].len_actual > 180);
//      }
//
//      if (score_tmp > score_max)
//      {
//          score_max = score_tmp;
//          combo_best = combo_tmp;
//      }
//  }
//
//  // 2. stop the find
//  if (combo_tmp.size() >= sz_thres)
//  {
//      return 0;
//  }
//
//  // 3. go on the find
//  for (int i = level + 1; i < sz; ++i)
//  {
//      int flag = 0;
//      if (level >= 0)
//      {
//          flag = weights[level][i];
//
//          if (statue + flag >= 1)
//          {
//              continue;
//          }
//      }
//
//      combo_tmp.push_back(_lanes[i]);
//      find_best_combo(combo_best, combo_tmp, weights, score_max, i, statue + flag, sz, sz_thres);
//      combo_tmp.pop_back();
//  }
//
//  return 0;
//}

int GroundDetection::calc_lane_line_intersection(
    const cv::Point2f& start_pt1,
    const cv::Point2f& end_pt1,
    const cv::Point2f& start_pt2,
    const cv::Point2f& end_pt2,
    cv::Point2f& insection_point) {
    insection_point = cv::Point2f(0.0f, 0.0f);
    // 1. calculate the intersection of two lanes
    float a = end_pt1.x - start_pt1.x;
    float b = start_pt2.x - end_pt2.x;
    float c = end_pt1.y - start_pt1.y;
    float d = start_pt2.y - end_pt2.y;
    float g = start_pt2.x - start_pt1.x;
    float h = start_pt2.y - start_pt1.y;
    float f = a * d - b * c;

    if (fabs(f) < FLOAT_MIN) {
        return -1;
    }

    float t = (d * g - b * h) / f;
    insection_point = start_pt1 + (end_pt1 - start_pt1) * t;
    return 0;
}

int GroundDetection::generate_lane_line_detections_fine(int combo_sz) {
    // 1. nms
    std::sort(_lanes.begin(), _lanes.end(), sort_by_radium_asc);

    for (int i = 1; i < int(_lanes.size()) - 1; ++i) {
        float theta_diff1 = _lanes[i - 1].theta_ipm - _lanes[i].theta_ipm;
        float theta_diff2 = _lanes[i + 1].theta_ipm - _lanes[i].theta_ipm;

        if (fabs(theta_diff1) > CV_PI * 10 / 180
                && fabs(theta_diff2) > CV_PI * 10 / 180
                && theta_diff1 * theta_diff2 > 0) {
            _lanes.erase(_lanes.begin() + i);
            --i;
        }
    }

    for (int i = 0; i < int(_lanes.size()) - 1; ++i) {
        float theta_diff = _lanes[i + 1].theta_ipm - _lanes[i].theta_ipm;
        float radium_diff = _lanes[i + 1].radium_ipm - _lanes[i].radium_ipm;

        if (fabs(radium_diff) < 40
                && fabs(theta_diff) < CV_PI * 5 / 180) {
            float score1 = _lanes[i + 1].conf + _lanes[i + 1].occupy_ratio;
            float score2 = _lanes[i].conf + _lanes[i].occupy_ratio;

            if (score1 > score2) {
                _lanes.erase(_lanes.begin() + i);
                --i;
            } else {
                _lanes.erase(_lanes.begin() + i + 1);
                --i;
            }
        }
    }

    // 2. calc the distance to the camera
    int sz = _lanes.size();

    //std::cout << "height: " << _roi_out.height << std::endl;
    for (int i = 0; i < sz; ++i) {
        cv::Vec4f vec;
        vec = _lanes[i].vec_ipm;
        // _roi_out.height
        _lanes[i].dist_to_camera = -(-vec[0] * (_camera_pos_ipm.y - vec[2]) + vec[1] *
                                     (_camera_pos_ipm.x + _camera_shift_x - vec[3]));
        //std::cout << "current center x: " << _camera_pos_ipm.x << std::endl;
        //std::cout << vec[0] << "  " << vec[1] << "  " << vec[2] << "  " << vec[3] << std::endl;
        std::cout << i << "th dist_to_camera: " << _lanes[i].dist_to_camera << std::endl;
    }

    //0912 for display
    for (int i = 0; i < sz - 1; ++i) {
        if (_lanes[i].dist_to_camera * _lanes[i + 1].dist_to_camera < 0) {
            _ratio_display = fabs(_lanes[i].dist_to_camera) / (_lanes[i + 1].dist_to_camera -
                             _lanes[i].dist_to_camera);
        }
    }

    std::sort(_lanes.begin(), _lanes.end(), sort_by_dist_asc);
    // 3. update the kalman filter
    sz = _lanes.size();
    int count_sz = 0;
    cv::Point2f vanish_point = cv::Point2f(0.0f, 0.0f);

    for (int i = 0; i < sz; ++i) {
        for (int j = i + 1; j < sz; ++j) {
            cv::Point2f start_pt1 = _lanes[i].start_pt_img;
            cv::Point2f end_pt1 = _lanes[i].end_pt_img;
            cv::Point2f start_pt2 = _lanes[j].start_pt_img;
            cv::Point2f end_pt2 = _lanes[j].end_pt_img;
            // 3.1. calculate the intersection
            cv::Point2f pt_img;

            if (calc_lane_line_intersection(start_pt1, end_pt1, start_pt2, end_pt2, pt_img)) {
                continue;
            }

            // 3.2. if the intersection is in the image, the fork; or the vanish point
            //pt_img.y -= _roi_in.y;
            cv::Point2f pt_ipm = _ipm_transformer.pt_transformation(pt_img,
                                 _ipm_transformer.get_img_to_ipm());

            if (pt_ipm.x < _roi_out.width && pt_ipm.x >= 0
                    && pt_ipm.y < _roi_out.height && pt_ipm.y >= 0) {
            } else if (fabs(pt_img.x - _vanish_point.x) <= 100
                       && fabs(pt_img.y - _vanish_point.y) <= 100) {
                vanish_point += pt_img;
                ++count_sz;
            }
        }
    }

    if (count_sz > 0) {
        std::vector<float> measure_vals;
        measure_vals.push_back(vanish_point.x / count_sz);
        measure_vals.push_back(vanish_point.y / count_sz);
        _kalman_vanish_point->do_update(measure_vals);
    }

    // 4. re-arrange the begin and end point of the lane lines
    for (int i = 0; i < _lanes.size(); ++i) {
        cv::Vec4f vec = _lanes[i].vec_ipm;
        _lanes[i].start_pt_ipm = cv::Point2f(_ransac_fitting.from_y_to_x(vec, 0), _roi_out.y);
        _lanes[i].end_pt_ipm = cv::Point2f(_ransac_fitting.from_y_to_x(vec, _roi_out.height),
                                           _roi_out.height);
        _lanes[i].start_pt_img = _ipm_transformer.pt_transformation(_lanes[i].start_pt_ipm,
                                 _ipm_transformer.get_ipm_to_img());
        _lanes[i].end_pt_img = _ipm_transformer.pt_transformation(_lanes[i].end_pt_ipm,
                               _ipm_transformer.get_ipm_to_img());
        //_lanes[i].start_pt_img.y += _roi_in.y;
        //_lanes[i].end_pt_img.y += _roi_in.y;
    }

    // 5. remove the combo with the single lane line
    if (_lanes.size() <= 1) {
        _lanes.clear();
    }

    return 0;
}

//
//
//int GroundDetection::analyze_topology()
//{
//  // 1. for each lane, calculate the dist to the left and right
//  int sz = _lanes.size();
//  for (int i = 0; i < sz; ++i)
//  {
//      // 1.1. the left lane
//      if (_lanes[i].dist_to_camera <= 0)
//      {
//          float avg_dist = 0.0f;
//          for (int j = 0; j < _left_curb.size(); ++j)
//          {
//              float dist = _ransac_fitting.get_dist(_lanes[i].vec_ipm,
//                  cv::Point2f(_left_curb[j].y, _left_curb[j].x));
//              avg_dist += dist;
//          }
//
//          if (avg_dist < 0.00001 || _left_curb.size() < 50)
//          {
//              _left_lane_idx = -1;
//          }
//          else
//          {
//              _left_lane_idx = avg_dist / _left_curb.size() / 60 + 1.1;
//          }
//
//      }
//      else
//      {
//          float avg_dist = 0.0f;
//          for (int j = 0; j < _right_curb.size(); ++j)
//          {
//              float dist = _ransac_fitting.get_dist(_lanes[i].vec_ipm,
//                  cv::Point2f(_right_curb[j].y, _right_curb[j].x));
//              avg_dist += dist;
//          }
//
//          if (avg_dist < 0.00001 || _right_curb.size() < 50)
//          {
//              _right_lane_idx = -1;
//          }
//          else
//          {
//              _right_lane_idx = avg_dist / _right_curb.size() / 60 + 1.1;
//          }
//      }
//  }
//
//  return 0;
//}

int GroundDetection::show_image(const cv::Mat& im, const std::string win_name) {
    //#ifndef __GNUC__
    if (1) {
        cv::Mat im_draw;

        if (im.channels() == 1) {
            cv::Mat im2 = im.clone();
            im2.convertTo(im2, CV_8UC1);
            cv::cvtColor(im, im_draw, CV_GRAY2BGR);
        } else {
            im.copyTo(im_draw);
        }

        for (int i = 0; i < _lanes.size(); ++i) {
            //for apollo *** std::cout << "        [-] line " << (i + 1) << ": " << _lanes[i].start_pt_ipm << ", " << _lanes[i].end_pt_ipm << std::endl;
            for (int j = 0; j < _lanes[i].pts_ipm.size(); ++j) {
                cv::circle(im_draw, _lanes[i].pts_ipm[j].pt, 3, cv::Scalar(0, 255, 255), 1);
            }

            cv::line(im_draw, _lanes[i].start_pt_ipm, _lanes[i].end_pt_ipm,
                     cv::Scalar(0, 0, 255), 2);
            char texts[1024];
#ifdef __GNUC__
            //snprintf(texts, 1024, "%1.2f_%d", _lanes[i].conf, _lanes[i].type);
            snprintf(texts, 1024, "%d", _lanes[i].type);
#else
            sprintf_s(texts, "%d-%d-%1.2f", _lanes[i].type, _lanes[i].len_actual, _lanes[i].conf);
#endif
            cv::Point2f pt;
            pt.x = (_lanes[i].start_pt_ipm.x + _lanes[i].end_pt_ipm.x) / 2;
            pt.y = (_lanes[i].start_pt_ipm.y + _lanes[i].end_pt_ipm.y) / 2;
            cv::putText(im_draw, texts, pt, cv::FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(0, 255, 0), 1);
            cv::circle(im_draw, _lanes[i].fork_pt_ipm, 5, cv::Scalar(255, 0, 0), -1);
        }

        cv::line(im_draw, _left_curb.start_pt_ipm, _left_curb.end_pt_ipm,
                 cv::Scalar(0, 255, 0), 2);
        cv::line(im_draw, _right_curb.start_pt_ipm, _right_curb.end_pt_ipm,
                 cv::Scalar(0, 255, 0), 2);
        cv::circle(im_draw, cv::Point2f(_camera_pos_ipm.x, 223), 2, cv::Scalar(0, 0, 255), -1);
        //std::cout << "wihualea " << std::endl;
        //cv::circle(im_draw, cv::Point2f(_pt_show.x, 223), 2, cv::Scalar(0, 0, 255), -1);
        //cv::line(im_draw, _camera_pos_ipm, _pt_show, cv::Scalar(255, 255, 0), 2);
        //         char st_ratio[20];
        //
        //         sprintf(st_ratio, "%f", _ratio_display);
        std::string st_ratio = std::to_string(_ratio_display);
        cv::putText(im_draw, st_ratio, cv::Point(_camera_pos_ipm.x - 40, 210),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(0, 0, 255), 1);
        char texts[1024];

        //#ifdef __GNUC__
        //      snprintf(texts, 1024, "%d, %d", _left_lane_idx, _right_lane_idx);
        //#else
        //      sprintf_s(texts, "%d, %d", _left_lane_idx, _right_lane_idx);
        //#endif
        //      cv::putText(im_draw, texts, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX,
        //          0.5, cv::Scalar(255, 0, 255), 1);
        if (win_name == "refined") {
            char nam[100];
            snprintf(nam, 95, "image2/refined_%d.jpg", ++_count);
            cv::imwrite(nam, im_draw);
        }

#ifdef __GNUC__
        snprintf(texts, 1024, "%s.png", win_name.c_str());
        //      cv::imwrite(texts, im_draw);
        //cv::imshow(win_name, im_draw);
#else
        cv::imshow(win_name, im_draw);
#endif
    }

    //#endif
    return 0;
}

int GroundDetection::camexpara_autofit(cv::Point3f& expara_bias, 
std::vector<cv::Vec4f> ego_left_lines, FIXED_PARAM static_param) {
    static_param.camera_info.pitch -= 0.20;
    std::vector<cv::Vec4f> lane_lines;
    std::vector<cv::Vec4f> lane_lines_prop;
    cv::Vec4f lane_line;
    cv::Vec4f lane_line_prop;

    //std::cout << ego_left_lines.size() << " #11111111" << std::endl; 
    //filter useful lines 
    if (!ego_left_lines.empty()) {
        for (int i = 0; i < ego_left_lines.size(); ++i) {
            lane_line = ego_left_lines[i];
            lane_line_prop[0] = atan((lane_line[1] - lane_line[3]) /
                                     (lane_line[0] - lane_line[2])) * 180 / CV_PI;
            lane_line_prop[1] = sqrt((lane_line[1] - lane_line[3]) *
                                     (lane_line[1] - lane_line[3]) + (lane_line[0] - lane_line[2]) *
                                     (lane_line[0] - lane_line[2]));
            lane_line_prop[2] = 0;
            lane_line_prop[3] = 0;

            //if (lane_line_prop[1] > 1.5) {
                lane_lines.push_back(lane_line);
                lane_lines_prop.push_back(lane_line_prop);
            //}
        }
    }
    //std::cout << "size1: " << lane_lines_prop.size() << std::endl;
    //std::cout << " done filter " << std::endl;

    float lane_lines_meanslope = 0.0;
    float lane_lines_varslope = 0.0;
    float lane_lines_meanslopepre = 0.0;
    float lane_lines_varslopepre = 0.0;

    std::vector<cv::Vec4f> ipm_lane_lines;

    //int _itermax = 20;
    lane_lines_meanslopepre = lane_lines_meanslope;
    //lane_lines_varslopepre = lane_lines_varslope;
    lane_lines_varslope = 10000;
    double mean_dif = 0.0;
    double var_dif = lane_lines_varslope;
    int iternum = 0;
    if(ego_left_lines.size() >= 2) {
        while (1) {
            lane_lines_varslopepre = lane_lines_varslope;
            ipm_lane_lines.clear();
            //std::cout << "lane_lines_varslopepre = " << lane_lines_varslopepre << std::endl;
            iternum++;
            //if (iternum > _itermax) {
            //    break;
            //}

            static_param.camera_info.pitch += 0.01; //lane_lines_varslope / 10;
            //std::cout << static_param.camera_info.pitch << std::endl;
            _ipm_transformer.do_transformation(static_param.camera_info, 
            static_param.roi_in, static_param.roi_out);
            //std::cout << " 1111 loop " << std::endl; 
            for (int i = 0; i < ego_left_lines.size(); ++i) {
                cv::Vec4f ipm_lane_line;
                cv::Point2d src_pt1(ego_left_lines[i][0], ego_left_lines[i][1]);
                cv::Point2d src_pt2(ego_left_lines[i][2], ego_left_lines[i][3]);
                cv::Point2d dst_pt = src_pt1;
                //std::cout << "dst_pt " << dst_pt.x << "  " << dst_pt.y << std::endl;
                cv::Point2f dst = _ipm_transformer.pt_transformation(dst_pt,
                 _ipm_transformer.get_img_to_ipm());
                //std::cout << " in 1111 loop " << std::endl;
                //std::cout << "dst_pt " << dst.x << "  " << dst.y << std::endl;
                ipm_lane_line[0] = dst.x;
                ipm_lane_line[1] = dst.y;

                dst_pt = src_pt2;
                dst = _ipm_transformer.pt_transformation(dst_pt, _ipm_transformer.get_img_to_ipm());
                ipm_lane_line[2] = dst.x;
                ipm_lane_line[3] = dst.y;
                ipm_lane_lines.push_back(ipm_lane_line);
            }

            //std::cout << " first loop " << std::endl;
            for (int i = 0; i < ego_left_lines.size(); ++i) {
                lane_line = ipm_lane_lines[i];
                //std::cout << " pppp: " << lane_line[0] << "  " << lane_line[1] << "  " << lane_line[2] << "  " << lane_line[3] << std::endl;
                lane_lines_prop[i][0] = atan((lane_line[1] - lane_line[3]) /
                                             (lane_line[0] - lane_line[2])) * 180 / CV_PI;
                if (lane_lines_prop[i][0] < 0) {
                    lane_lines_prop[i][0] += 180;
                }
                lane_lines_prop[i][1] = sqrt((lane_line[1] - lane_line[3]) *
                (lane_line[1] - lane_line[3]) + (lane_line[0] - lane_line[2]) *
                (lane_line[0] - lane_line[2]));
                lane_lines_prop[i][2] = 0;
                lane_lines_prop[i][3] = 0;
            }
            //std::cout << " second loop " << std::endl;
            //std::cout << lane_lines_prop.size() << "nnn,n,n" << std::endl;
            lane_lines_meanslope = 0;

            for (int i = 0; i < ipm_lane_lines.size(); ++i) {
                lane_lines_meanslope += lane_lines_prop[i][0];
                //std::cout << "k: "<< lane_lines_prop[i][0] << std::endl;
            }

            lane_lines_meanslope = lane_lines_meanslope / ipm_lane_lines.size();

            lane_lines_varslope = 0;
            for (int i = 0; i < ipm_lane_lines.size(); ++i) {
                lane_lines_varslope += fabs(lane_lines_prop[i][0] - lane_lines_meanslope);
            }

            lane_lines_varslope = lane_lines_varslope / lane_lines_prop.size();
            var_dif = lane_lines_varslopepre - lane_lines_varslope;
            //std::cout << "var_diff:: " << var_dif << std::endl;
            if (iternum > 40 || lane_lines_varslope < 0.8 ||  var_dif < 0) {
            //if (iternum > 10) {
                break;
            }

        }//end pitch loop.

        expara_bias.x = static_param.camera_info.pitch - 0.01;
        //std::cout << expara_bias.x << " pitch 090909090" << std::endl;
    }
    //std::cout << "endddd" << std::endl;
    return 0;
}

int GroundDetection::do_lane_detection(
    RETINFO& ret_info,
    FIXED_PARAM static_param) {
    // 1. the basic setup
    _lanes.clear();
    _left_curb.pts_ipm.clear();
    _right_curb.pts_ipm.clear();
    // 2. filter the image
    //for apollo *** std::cout << "        [-] Begin Filter" << std::endl;
    _timer.do_start_timer();
    filter_image();
    generate_driving_area();
    //fk apolo  _timer.do_end_timer("        [-] End Filter, misc time: ");
    // 3. warp the white mask
    //for apollo *** std::cout << "        [-] Begin Finding Curbs" << std::endl;
    _timer.do_start_timer();
    generate_curbs();
    //fk apolo  _timer.do_end_timer("        [-] End Finding Curbs, misc time: ");
    // 4. find the proposals
    //for apollo *** std::cout << "        [-] Begin Finding Proposals" << std::endl;
    _timer.do_start_timer();
    generate_lane_lines_coarse();
    show_image(_ipm_ori, "candidate");
    //fk apolo  _timer.do_end_timer("        [-] End Finding Proposals, misc time: ");
    // 3. analyze the topology
    //for apollo *** std::cout << "        [-] Begin Merge & Refine" << std::endl;
    _timer.do_start_timer();
    //std::cout << "size coarse: " << _lanes.size() << std::endl;
    generate_lane_lines_merged();
    //std::cout << "size merge: " << _lanes.size() << std::endl;
    show_image(_ipm_ori, "merge");
    generate_lane_line_detections_fine(6);
    //std::cout << "size fine: " << _lanes.size() << std::endl;
#ifdef use_autofit
    //0912 modify pitch for lines
    cv::Point3f expara_bias;
    std::vector<cv::Vec4f> lines;
    int start = 0, end = _lanes.size();

    //step 1 :  select center 2 or 3 lines
    if (_lanes.size() >= 5) {
        start = 0;
        end = 4;
    } else if (_lanes.size() == 4) {
        start = 0;
        end = 3;
    }

    //step 2 :  for each, trans each start and end point back to initial point
    for (int i = start; i < end; ++i) {
        //    int z = _lanes[i].pts_ipm.size() - 1;
        //cv::Vec4f pt(_lanes[i].pts_ipm[0].pt.x, _lanes[i].pts_ipm[0].pt.y, _lanes[i].pts_ipm[z].pt.x, _lanes[i].pts_ipm[z].pt.y);
        //std::cout << "ffffff_ipm0: " << _lanes[i].start_pt_ipm.x << "\t" << _lanes[i].start_pt_ipm.y << "\t" << _lanes[i].end_pt_ipm.x << "\t" << _lanes[i].end_pt_ipm.y << std::endl;
        cv::Point2f ptst = _ipm_transformer.pt_transformation(_lanes[i].start_pt_ipm,
                           _ipm_transformer.get_ipm_to_img());
        cv::Point2f pted = _ipm_transformer.pt_transformation(_lanes[i].end_pt_ipm,
                           _ipm_transformer.get_ipm_to_img());
        //std::cout << "ffffff_img0: " << _lanes[i].start_pt_ipm.x << "\t" << _lanes[i].start_pt_ipm.y << "\t" << _lanes[i].end_pt_ipm.x << "\t" << _lanes[i].end_pt_ipm.y << std::endl;
        cv::Vec4f pt(ptst.x, ptst.y, pted.x, pted.y);
        lines.push_back(pt);
    }

    //std::cout << "ptt00: " << _camera_pos_ipm.x << "\t" << _camera_pos_ipm.y << std::endl;
    //cv::Point2f ptt0 = _ipm_transformer.pt_transformation(_camera_pos_ipm, _ipm_transformer.get_ipm_to_img());
    //_camera_pos_ipm = ptt0;
    //std::cout << "ptt01: " << _camera_pos_ipm.x << "\t" << _camera_pos_ipm.y << std::endl;
    //step 3:  autofti
    //do transformation to _lanes inner
    int nm = camexpara_autofit(expara_bias, lines, static_param);
    std::cout << expara_bias.x << " ##big " << std::endl;
    _true_pitch = expara_bias.x;
#endif
    show_image(_ipm_ori, "refined");
    ret_info.lanes_det = _lanes;
    ret_info.left_curb = _left_curb;
    ret_info.right_curb = _right_curb;
    ret_info.vanish_point = _vanish_point;
    cv::cvtColor(_ipm_mask * 50, ret_info.mask, CV_GRAY2BGR);
    //_ipm_driving_area.copyTo(ret_info.road_available_ipm);
    //cv::warpPerspective(_ipm_driving_area, ret_info.road_available_img,
    //  _ipm_transformer.get_ipm_to_img(),
    //  cv::Size(_img_ori.cols, _img_ori.rows));
    //cvtColor(_ipm_mask * 50, ret_info.mask, cv::COLOR_GRAY2BGR);
    return 0;
}

bool sort_by_y_desc(const cv::Point2d& p1, const cv::Point2d& p2) {
    return p1.y < p2.y;
}

int GroundDetection::do_arrow_detection(
    RETINFO& ret_info) {
    //fk apolo std::cout << "        [-] Begin MSER" << std::endl;
    _timer.do_start_timer();
    // 2. use CNN, preserve arrows without overlap or on the boundary
    cv::Mat ipm_bw2 = _ipm_mask == CLS_ARROW;
    cv::Mat ipm_bw2_tmp = ipm_bw2.clone();
    cv::Mat ipm_bg = (_ipm_mask == CLS_CAR) | (_ipm_mask == CLS_BG);
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(ipm_bg, ipm_bg, cv::MORPH_DILATE, element);
    //cv::imshow("bg", ipm_bg);
    cv::Rect roi_arrows = cv::Rect(INT_MAX, INT_MAX, INT_MIN, INT_MIN);
    std::vector<std::vector<cv::Point> > contours1;
    findContours(ipm_bw2_tmp, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    for (int i = 0; i < contours1.size(); ++i) {
        bool flag = true;

        for (int j = 0; j < contours1[i].size(); ++j) {
            // 2.1. if overlap or on the boundary, neglect it
            if (ipm_bg.at<uchar>(contours1[i][j])
                    || contours1[i][j].x == 0 || contours1[i][j].x == _roi_out.width - 1
                    || contours1[i][j].y == 0 || contours1[i][j].y == _roi_out.height - 1) {
                cv::drawContours(ipm_bw2, contours1, i, cv::Scalar(0), -1);
                flag = false;
                break;
            }
        }

        // 2.2. if arrow, refine the roi
        if (flag) {
            cv::Rect rt = cv::boundingRect(contours1[i]);
            roi_arrows.x = std::min(roi_arrows.x, rt.x);
            roi_arrows.y = std::min(roi_arrows.y, rt.y);
            roi_arrows.width = std::max(roi_arrows.width, rt.x + rt.width);
            roi_arrows.height = std::max(roi_arrows.height, rt.y + rt.height);
        }
    }

#ifdef OPENCV300
    // 1. use mser
    std::vector<std::vector<cv::Point> > regions;
    regions.clear();
    std::vector<cv::Rect > rects;

    //mser->detectRegions(_ipm_gray, regions, rects);
    //_timer.do_end_timer("        [-] End MSER, misc time: ");

    if (roi_arrows.height - roi_arrows.y >= 30
            && roi_arrows.width - roi_arrows.x >= 60) {
        cv::Ptr<cv::MSER> mser = cv::MSER::create(4, 200, 2000, 0.25, 0.2);
        //cv::imshow("sb", _ipm_gray(cv::Rect(roi_arrows.x, roi_arrows.y,
        //  roi_arrows.width - roi_arrows.x, roi_arrows.height - roi_arrows.y)));
        mser->detectRegions(_ipm_gray(cv::Rect(roi_arrows.x, roi_arrows.y,
                                               roi_arrows.width - roi_arrows.x,
                                               roi_arrows.height - roi_arrows.y)), regions, rects);
        //fk apolo      _timer.do_end_timer("        [-] End MSER, misc time: ");
    }

#else
    std::vector<std::vector<cv::Point> > regions;
    regions.clear();
    //ms(_ipm_gray, regions, cv::Mat());

    //_timer.do_end_timer("        [-] End MSER, misc time: ");

    if (roi_arrows.height >= 20
            && roi_arrows.width >= 60) {
        cv::MSER ms(4, 200, 2000, 0.25, 0.2);
        //cv::imshow("sb", _ipm_gray(cv::Rect(roi_arrows.x, roi_arrows.y,
        //  roi_arrows.width - roi_arrows.x, roi_arrows.height - roi_arrows.y)));
        ms(_ipm_gray(cv::Rect(roi_arrows.x, roi_arrows.y,
                              roi_arrows.width - roi_arrows.x,
                              roi_arrows.height - roi_arrows.y)), regions, cv::Mat());
        //fk apolo      _timer.do_end_timer("        [-] End MSER, misc time: ");
    }

#endif
    cv::Mat ipm_bw1 = cv::Mat::zeros(_roi_out.height, _roi_out.width, CV_8U);
    int sz_bw1 = regions.size();

    for (int i = 0; i < sz_bw1; ++i) {
        int sz_region = regions[i].size();

        for (int j = 0; j < sz_region; ++j) {
            ipm_bw1.at<uchar>(regions[i][j].y + roi_arrows.y, regions[i][j].x +
                              roi_arrows.x) = 255;
        }
    }

    //fk aplol  _timer.do_end_timer("        [-] End MSER, misc time: ");
    //
    //fk apolo std::cout << "        [-] Begin Finding Arrows" << std::endl;
    _timer.do_start_timer();
    // 3. fusion masks to generate the arrows
    cv::Mat ipm_bw = ipm_bw1 & ipm_bw2;
    cv::Mat im_bw_tmp = ipm_bw.clone();
    std::vector<std::vector<cv::Point> > contours;
    findContours(im_bw_tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // 2. scan each contour, generate the candidate lanes
    _arrows.clear();
    cv::Mat im_draw = _ipm_ori.clone();

    for (int i = 0; i < contours.size(); ++i) {
        cv::Rect rt = cv::boundingRect(contours[i]);

        if (rt.height < 20 || rt.height > 80
                || rt.width > 80 || rt.width < 15
                //|| rt.area() < 1000 || rt.area() > 4000
                || rt.y < _roi_out.height / 4
                || rt.y + rt.height >= _roi_out.height - 4) {
            continue;
        }

        //cv::drawContours(im_draw, contours, i, cv::Scalar(0, 0, 255), 2);
        ARROW arrow;
        int sz_contour = contours[i].size();
        arrow.pts_ipm.resize(sz_contour);
        arrow.pts_img.resize(sz_contour);

        for (int j = 0; j < sz_contour; ++j) {
            arrow.pts_ipm[j] = contours[i][j]; /*_ipm_transformer.pt_transformation(contours[i][j],
                _ipm_transformer.get_ipm_to_img());*/
            arrow.pts_img[j] = _ipm_transformer.pt_transformation(contours[i][j],
                               _ipm_transformer.get_ipm_to_img());
            //arrow.pts_img[j].y += _roi_in.y;
        }

        std::vector<cv::Point2d> pts = arrow.pts_ipm;
        std::sort(pts.begin(), pts.end(), sort_by_y_desc);
        int sz = std::min(int(pts.size()), 3);
        float avg_y_min = 0.0f;

        for (int j = 0; j < sz; ++j) {
            avg_y_min += pts[j].y;
        }

        arrow.top_line = avg_y_min / sz;
        sz = std::min(int(pts.size()), 5);
        float avg_y_max = 0.0f;

        for (int j = pts.size() - 1; j >= pts.size() - sz; --j) {
            avg_y_max += pts[j].y;
        }

        arrow.bottom_line = avg_y_max / sz;
        _arrows.push_back(arrow);
    }

    ret_info.arrows_det = _arrows;
    //fk apolo  _timer.do_end_timer("        [-] End Finding Arrows, misc time: ");
    //cv::imshow("sss", im_draw);
    //cv::waitKey(0);
    return 0;
}

SYSTEM_STATUS GroundDetection::do_ground_detection(
    const cv::Mat& img_ori,
    const cv::Mat& ipm_ori,
    const cv::Mat& ipm_gray,
    const cv::Mat& ipm_mask,
    const std::vector<cv::Mat>& ipm_probs,
    const IPMTransformer& ipm_transformer,
    const DYNAMIC_PARAMS& param,
    RETINFO& ret_info,
    FIXED_PARAM& static_param) {
    //fk apolo std::cout << "    [+] Begin Ground Detection" << std::endl;
    //fk apolo std::cout << "        [-] Begin Data Transformation" << std::endl;
    _timer.do_start_timer();
    // 1. copy data
#ifdef remove_clone
    _img_ori = img_ori;
    _ipm_ori = ipm_ori;
    _ipm_gray = ipm_gray;
    _ipm_mask = ipm_mask;
#else
    _img_ori = img_ori.clone();
    _ipm_ori = ipm_ori.clone();
    _ipm_gray = ipm_gray.clone();
    _ipm_mask = ipm_mask.clone();
#endif
    
    _ipm_probs = ipm_probs;
    _ipm_transformer = ipm_transformer;
    _scene = param.hadmap_scene;
    //cv::imshow("_ipm_mask", _ipm_mask * 50);
    cv::Mat im_one = cv::Mat::ones(_img_ori.rows/*_roi_in.height - _roi_in.y*/,
                                   _img_ori.cols, CV_8UC1);
    cv::warpPerspective(im_one, _ipm_warp_mask, _ipm_transformer.get_img_to_ipm(),
                        cv::Size(_roi_out.width, _roi_out.height), cv::INTER_NEAREST);
    cv::Mat ipm_one = _ipm_warp_mask & (_ipm_mask != CLS_CAR);
#ifdef localization_display
    cv::imshow("_ipm_warp_mask", _ipm_warp_mask * 50);
    cv::imshow("ipm_one", ipm_one * 50);
#endif
    cv::reduce(ipm_one, _hist_ipm, 0, CV_REDUCE_SUM, CV_32FC1);
    cv::reduce(_ipm_warp_mask, _hist_ipm_all, 0, CV_REDUCE_SUM, CV_32FC1);
    //_camera_pos_img.y -= _roi_in.y;
    _camera_pos_img.y = 1207;
    //std::cout << "%%%%%" << _camera_pos_img.x << "  " << _camera_pos_img.y << std::endl;
    _camera_pos_ipm = _ipm_transformer.pt_transformation(_camera_pos_img,
                      _ipm_transformer.get_img_to_ipm());
    //std::cout << "*****" << _camera_pos_ipm.x << "  " << _camera_pos_ipm.y << std::endl;
    //fk aplo   _timer.do_end_timer("        [-] End Data Transformation, misc time: ");
    do_lane_detection(ret_info, static_param);
    do_arrow_detection(ret_info);
    //fk apolo std::cout << "    [-] End Ground Detection" << std::endl;
    //0917 modi pitch
#ifdef use_autofit
    if (fabs(static_param.camera_info.pitch - _true_pitch) < 0.3) { 
        static_param.camera_info.pitch = _true_pitch;
    }
#endif
    return SYS_GROUND_MATCH_Y;
}

#endif
