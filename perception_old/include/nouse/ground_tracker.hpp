#ifndef LOCALIZATION_GROUND_TRACKER_H
#define LOCALIZATION_GROUND_TRACKER_H

#include "kalman.hpp"
//#include "hungarian_alg.hpp"
#include "ipm.hpp"
#include "basic_struct.h"
//#include <unistd.h>
#include "config.h"

class GroundTracker {
public:
    GroundTracker() {
        _kalman_yaw_angle = NULL;
    }
    ~GroundTracker() {
        if (_kalman_yaw_angle) {
            delete _kalman_yaw_angle;
            _kalman_yaw_angle = NULL;
        }
    }

public:
    int do_init(
        const cv::Rect& roi_out,
        const float& shift_y) {
        _roi_out = roi_out;
        _shift_y = shift_y;

        //_max_missing_det = 1;

        _width_versus_height = -1.0f;

        _former_min_distance_to_line = -1; //xhzhu 0715 chedao baochi
        _former_channel_index = -1;
        _former_shift_x = -1;
        _min_distance_side = -1;
        _former_angle = -1;

        _former_ratio = -1;
        _change_count = 0;
        return 0;
    }
    //int do_ground_tracker(
    //    RETINFO &ret_info,
    //    std::vector<LANE_COMBO> lanes_combo);
    SYSTEM_STATUS do_delta_x_tracker(
        const IPMTransformer& ipm_transformer,
        RETINFO& ret_info);
    SYSTEM_STATUS do_delta_y_tracker(
        const IPMTransformer& ipm_transformer,
        RETINFO& ret_info);
private:
    //std::vector<LANE_TEMPORAL> _lanes_temporal;
    //std::vector<std::vector<LANE> > _lanes_records;
    //int _max_missing_det;
    cv::Rect _roi_out;
    IPMTransformer _ipm_transformer;
    //std::vector<bool> _is_near_right_flags;

    Kalman* _kalman_yaw_angle;

    float _width_versus_height;

    //std::vector<float> _shift_xs;
    //std::vector<int> _borders;

    Timer _timer;
    float _shift_y;

    double _former_min_distance_to_line; //xhzhu 0715 for chedao keep
    int _former_channel_index;  //xhzhu 0715 for chedao keep
    double _former_shift_x;
    int _min_distance_side;//xhzhu 0808 keep
    cv::Point2d _former_basis;
    double _former_angle;

    std::deque<double> _old_ratios; //0906
    double _former_ratio;
    int _change_count;
};

int find_best_combo(
    const std::vector<std::vector<double> > cost_matrix,
    std::vector<int>& assignments,
    std::vector<int>& assignments_max,
    float& cost_min,
    int k,
    int last_assignment,
    const int& gt_num,
    const int& detect_num) {
    // 1. if scan all gt, quit
    if (k == gt_num) {
        // 1.1. calculate the score of all assignments
        float cost_tmp = 0.0f;

        for (int i = 0; i < gt_num; ++i) {
            // 1.1.1. penalty for not assigned track
            if (assignments[i] == -1) {
                cost_tmp += 60.0f;
            } else {
                cost_tmp += cost_matrix[i][assignments[i]];
            }
        }

        // 1.2. refine the minimal cost
        if (cost_tmp < cost_min) {
            cost_min = cost_tmp;
            assignments_max = assignments;
        }

        return 0;
    }

    // 2. find next element; detect_num mean -1 assignment
    for (int i = last_assignment + 1; i <= detect_num; ++i) {
        int next_last_assignment = (i == detect_num) ? (k == 0 ? -1 : assignments[k - 1]) : i;
        assignments[k] = (i == detect_num) ? -1 : i;
        find_best_combo(cost_matrix, assignments, assignments_max,
                        cost_min, k + 1, next_last_assignment, gt_num, detect_num);
        assignments[k] = -1;
    }

    return 0;
}

bool sort_by_dist_fabs_asc(const LANE& l1, const LANE& l2) {
    return fabs(l1.dist_to_camera) < fabs(l2.dist_to_camera);
}

SYSTEM_STATUS GroundTracker::do_delta_x_tracker(
    const IPMTransformer& ipm_transformer,
    RETINFO& ret_info
) {
    ret_info.match_x = -1;
    ret_info.long_x = -1;
    ret_info.short_x = -1;
    ret_info.final_x = -1;
    //ret_info.rotate_angle = -1;
    //fk apolo    std::cout << "    [+] Begin Delta-x Tracker" << std::endl;
    _timer.do_start_timer();

    // 1. the parameter setup
    _ipm_transformer = ipm_transformer;
    std::vector<LANE> lanes_det = ret_info.lanes_det;
    std::vector<LANE_COMBO>& lanes_combo = ret_info.lanes_combo;

    if (_width_versus_height < 0) {
        _width_versus_height = _ipm_transformer.get_width_versus_height();
    }

    //fk apolo    _timer.do_end_timer("    [-] End Delta-x Tracker, misc time: ");

    // 2. construct the m*n pairs
    int num_detects = lanes_det.size();
    int num_tracks = lanes_combo[0].lanes.size();
    std::cout << "detect num:  " << num_detects << "  hadmap num:  " << num_tracks << std::endl;

    std::vector<int> assignments_global(0);
    float cost_min_global = INT_MAX;

    std::vector<int> assignments_global_bakeup(0);
    float cost_min_global_bakeup = INT_MAX;
    double x_short = 1;

    for (int idx = 0; idx < num_detects; ++idx) {
        for (int idx2 = 0; idx2 < num_tracks; ++idx2) {
            lanes_det[idx].id = idx;

            // 2.1. match the idx detect and the idx2 track

            ret_info.shift_pos = lanes_det[idx].end_pt_ipm.x
                                 - lanes_combo[0].lanes[idx2].end_pt_ipm.x / x_short;

            std::vector<LANE> lanes_det_tmp = lanes_det;

            for (int i = 0; i < lanes_det_tmp.size(); ++i) {
                lanes_det_tmp[i].end_pt_ipm.x -= ret_info.shift_pos;
                lanes_det_tmp[i].start_pt_ipm.x -= ret_info.shift_pos;
            }

            LANE left_curb = ret_info.left_curb;
            LANE right_curb = ret_info.right_curb;
            left_curb.end_pt_ipm.x -= ret_info.shift_pos;
            left_curb.start_pt_ipm.x -= ret_info.shift_pos;
            right_curb.start_pt_ipm.x -= ret_info.shift_pos;
            right_curb.end_pt_ipm.x -= ret_info.shift_pos;
            left_curb.end_pt_ipm.x = std::min(left_curb.end_pt_ipm.x, left_curb.start_pt_ipm.x);
            right_curb.end_pt_ipm.x = std::max(right_curb.end_pt_ipm.x, right_curb.start_pt_ipm.x);

            // 2.2. construct the cost_matrix
            std::vector<std::vector<double> > costs(num_tracks,
                                                    std::vector<double>(num_detects));

            for (int i = 0; i < num_tracks; ++i) {
                for (int j = 0; j < num_detects; ++j) {
                    costs[i][j] = fabs(lanes_det_tmp[j].end_pt_ipm.x
                                       - lanes_combo[0].lanes[i].end_pt_ipm.x)
                                  + fabs(lanes_det_tmp[j].occupy_ratio
                                         - lanes_combo[0].lanes[i].type) * 20
                                  + (lanes_det_tmp[j].type == 1
                                     && lanes_combo[0].lanes[i].type == 0) * 100000;
                }
            }

            // 2.3. find the best assignments
            std::vector<int> assignments(num_tracks, -1);
            std::vector<int> assignments_max;
            float cost_min = INT_MAX;

            find_best_combo(costs, assignments, assignments_max, cost_min,
                            0, -1, num_tracks, num_detects);

            // 2.4. if the left lane is on the right of the camera, wrong combo
            if (assignments_max[0] != -1
                    && lanes_det_tmp[assignments_max[0]].dist_to_camera >= 0) {
                continue;
            }

            if (assignments_max[num_tracks - 1] != -1
                    && lanes_det_tmp[assignments_max[num_tracks - 1]].dist_to_camera <= 0) {
                continue;
            }

            if (cost_min < cost_min_global_bakeup) {
                cost_min_global_bakeup = cost_min;
                assignments_global_bakeup = assignments_max;
            }

            if (cost_min < cost_min_global) {

                // 2.2. if the track left is more left than the left_curb, continue; right the same
                if (left_curb.pts_ipm.size() > 0
                        && std::min(left_curb.end_pt_ipm.x, lanes_det_tmp[0].end_pt_ipm.x - 60)
                        > lanes_combo[0].lanes[0].end_pt_ipm.x) {
                    continue;
                }

                if (right_curb.pts_ipm.size() > 0
                        && std::max(right_curb.end_pt_ipm.x,
                                    lanes_det_tmp[num_detects - 1].end_pt_ipm.x + 60)
                        < lanes_combo[0].lanes[num_tracks - 1].end_pt_ipm.x) {
                    continue;
                }

                cost_min_global = cost_min;
                assignments_global = assignments_max;
            }

        }

    }


    //fk apolo    _timer.do_end_timer("    [-] End Delta-x Tracker, misc time: ");
    //std::cout << "match end "<< std::endl;

    if (assignments_global.size() == 0) {
        assignments_global = assignments_global_bakeup;
    }

    for (int i = 0; i < assignments_global.size(); ++i) {
        std::cout << "        [-] " << i << " is assigned to " << assignments_global[i]
                  << std::endl;
    }

    // 3. find the nearest corresponding
    std::sort(lanes_det.begin(), lanes_det.end(), sort_by_dist_fabs_asc);
    //std::cout << "sort end "<< std::endl;

    std::vector<float> shift_xs;
    std::vector<int> borders;

    std::vector<float> yaws;
    std::vector<float> delta_yaws;

    bool is_the_same_dir = false;

    if (lanes_det.size() >= 2) {
        is_the_same_dir =
            lanes_det[0].dist_to_camera * lanes_det[lanes_det.size() - 1].dist_to_camera < 0;
    }

    for (int i = 0; i < lanes_det.size(); ++i) {
        // 3.1. only matched one can be preserved
        for (int j = 0; j < assignments_global.size(); ++j) {
            if (lanes_det[i].id == assignments_global[j]) {
                if (shift_xs.size() == 1 && is_the_same_dir
                        && lanes_det[i].dist_to_camera * shift_xs[0] > 0) {
                    continue;
                }

                shift_xs.push_back(lanes_det[i].dist_to_camera);
                borders.push_back(j);
                lanes_combo[0].lanes[j].status = 1;

                float angle = atan2f((lanes_det[i].end_pt_ipm.x - lanes_det[i].start_pt_ipm.x),
                                     (lanes_det[i].end_pt_ipm.y - lanes_det[i].start_pt_ipm.y)
                                     * _width_versus_height);
                yaws.push_back(angle);
                delta_yaws.push_back(lanes_det[i].dist_to_camera);

            }
        }

        if (shift_xs.size() == 2) {
            break;
        }
    }

    //fk apolo    _timer.do_end_timer("    [-] End Delta-x Tracker, misc time: ");
    //std::cout << "find  border"<<  borders.size() << std::endl;
    // 4. refine the delta_x and yaw
    ret_info.delta_x = 0.5;
    ret_info.shift_pos = 0;
    std::cout << "before channel " << ret_info.channel_index << std::endl;

    //ret_info.pt_estimation_draw.x = 0; // xhzhu 0713
    //std::cout << "border size: " << borders.size()  << "  ##  boeders zhiqina, x zhi : " << ret_info.pt_estimation_draw.x << std::endl; //xhzhu 0713
    if (borders.size() == 2 && abs(shift_xs[0] - shift_xs[1]) > 0.00001) { //xhzhu 0713

        //0910 useful
        //std::cout << "        [-] distance to " << borders[0] << " = " << shift_xs[0] << std::endl;
        //std::cout << "        [-] distance to " << borders[1] << " = " << shift_xs[1] << std::endl;
        //std::cout << fabs(std::min(shift_xs[0], shift_xs[1])) << "  "
        //          << fabs(shift_xs[0] - shift_xs[1]) << std::endl;
        double ratio = fabs(std::min(shift_xs[0], shift_xs[1])) / fabs(shift_xs[0] - shift_xs[1]);
        //std::cout << "ratio:  " << ratio << std::endl;
        float shift_x = 0;

        if (_former_ratio == -1) {
            _former_ratio = ratio;
            shift_x = std::min(lanes_combo[0].lanes[borders[0]].end_pt_ipm.x,
                               lanes_combo[0].lanes[borders[1]].end_pt_ipm.x)
                      + abs(lanes_combo[0].lanes[borders[0]].end_pt_ipm.x
                            - lanes_combo[0].lanes[borders[1]].end_pt_ipm.x) * ratio;

            for (int i = 0; i < lanes_combo[0].lanes.size(); ++i) {
                if (lanes_combo[0].lanes[i].end_pt_ipm.x > shift_x) {
                    ret_info.channel_index = i;
                    break;
                }
            }

            _former_channel_index = ret_info.channel_index;
        } else {
            if (shift_xs[0] * shift_xs[1] <= 0) {
                _former_ratio = ratio;
                shift_x = std::min(lanes_combo[0].lanes[borders[0]].end_pt_ipm.x,
                                   lanes_combo[0].lanes[borders[1]].end_pt_ipm.x)
                          + abs(lanes_combo[0].lanes[borders[0]].end_pt_ipm.x
                                - lanes_combo[0].lanes[borders[1]].end_pt_ipm.x)
                          * ratio;

                for (int i = 0; i < lanes_combo[0].lanes.size(); ++i) {
                    if (lanes_combo[0].lanes[i].end_pt_ipm.x > shift_x) {
                        ret_info.channel_index = i;
                        break;
                    }
                }

                _former_channel_index = ret_info.channel_index;
            } else {
                ratio = _former_ratio;
                ret_info.channel_index = _former_channel_index;
                shift_x =
                    std::min(lanes_combo[0].lanes[ret_info.channel_index].end_pt_ipm.x,
                             lanes_combo[0].lanes[ret_info.channel_index - 1].end_pt_ipm.x)
                    + abs(lanes_combo[0].lanes[ret_info.channel_index].end_pt_ipm.x
                          - lanes_combo[0].lanes[ret_info.channel_index - 1].end_pt_ipm.x) * ratio;
            }
        }

        //std::cout << "ratio22: " << ratio << std::endl;
        //std::cout << ret_info.channel_index << std::endl;

        ret_info.pt_estimation_draw.x = shift_x;
        ret_info.match_x = shift_x;

        //std::cout << "border limian, x zhi : " << ret_info.pt_estimation_draw.x << std::endl; //xhzhu 0713
        //std::cout << lanes_combo[0].lanes[borders[0]].end_pt_ipm.x << " bihdhfwh### " <<  lanes_combo[0].lanes[borders[1]].end_pt_ipm.x << std::endl;
        //std::cout << borders[0] - borders[1] << "  chek " << shift_xs[0] << "  " << shift_xs[1] << std::endl;
        if (abs(borders[0] - borders[1]) == 1)
            //&& shift_xs[0] * shift_xs[1] < 0)
        {
            //std::cout << "biubiubiu " << std::endl;
            ret_info.pt_estimation_level.x = 0;
            //xhzhu 0816 for compare ratio
            ret_info.camera_dis_2_side.x = std::min(shift_xs[0], shift_xs[1]);
            ret_info.camera_dis_2_side.y = std::max(shift_xs[0], shift_xs[1]);
            ret_info.hadmap_dis_2_side.x = std::min(lanes_combo[0].lanes[borders[0]].end_pt_ipm.x,
                                                    lanes_combo[0].lanes[borders[1]].end_pt_ipm.x);
            ret_info.hadmap_dis_2_side.y = std::max(lanes_combo[0].lanes[borders[0]].end_pt_ipm.x,
                                                    lanes_combo[0].lanes[borders[1]].end_pt_ipm.x);
            //std::cout << ret_info.camera_dis_2_side.x << "  "
            //          << ret_info.camera_dis_2_side.y << "absd" << std::endl;
        } else {
            ret_info.pt_estimation_level.x = 1;
        }

        ret_info.shift_pos = shift_x;

        // 4.2. calc the yaw
        float yaw_new = yaws[0] * delta_yaws[1] / (delta_yaws[1] - delta_yaws[0])
                        - yaws[1] * delta_yaws[0] / (delta_yaws[1] - delta_yaws[0]);
        yaw_new = yaw_new * 180 / CV_PI;
        ret_info.yaw = ret_info.yaw_filtered - yaw_new;
        //std::cout << "$#$%#%#%#%#%: " << _former_min_distance_to_line << std::endl;



        // 4.3. refine the yaw kalman
        if (NULL == _kalman_yaw_angle) {
            // 4.3.1. new kalman filter
            _kalman_yaw_angle = new Kalman;
            std::vector<float> measure_vals;
            measure_vals.push_back(ret_info.yaw);
            _kalman_yaw_angle->init(1, 1, measure_vals, 1e-3, 1e-2);

            // 4.3.2. use curr results
            ret_info.yaw_filtered = ret_info.yaw;
        } else {
            // 4.3.3. use filtered results
            std::vector<float> measure_vals;
            measure_vals = _kalman_yaw_angle->do_predict();

            if (fabs(ret_info.yaw - measure_vals[0]) <= 0.5) {
                ret_info.yaw_filtered = ret_info.yaw;
            } else {
                ret_info.yaw_filtered = measure_vals[0];
            }

            if (fabs(ret_info.yaw - measure_vals[0]) <= 5) {
                // 4.3.4. refine the kalman filter
                measure_vals.clear();
                measure_vals.push_back(ret_info.yaw);
                _kalman_yaw_angle->do_update(measure_vals);
            }
        }
    }

    //std::cout << " channel index : " << ret_info.channel_index << std::endl;
    // 5. remove empty lane lines

    for (int i = 0; i < ret_info.lanes_det.size(); ++i) {
        // 7.1. if not assigned, set the status=-1, or refine the pitch angle
        std::vector<int>::iterator iter = std::find(assignments_global.begin(),
                                          assignments_global.end(), i);

        if (iter == assignments_global.end()) {
            ret_info.lanes_det[i].status = -1;
        }
    }

    return SYS_SUCCESS_X;
}

bool sort_by_center_asc(const ARROW& arrow1, const ARROW& arrow2) {
    float center1 = arrow1.bottom_line + arrow1.top_line;
    float center2 = arrow2.bottom_line + arrow2.top_line;
    return center1 < center2;
}

SYSTEM_STATUS GroundTracker::do_delta_y_tracker(
    const IPMTransformer& ipm_transformer,
    RETINFO& ret_info
) {
    //fk apolo std::cout << "    [+] Begin Delta-y Tracker" << std::endl;
    _timer.do_start_timer();

    int sz_arrows_det = ret_info.arrows_det.size();
    int sz_arrows = ret_info.arrows.size();
    ret_info.pt_estimation_draw.y = 0;

    if (sz_arrows_det > 1 && sz_arrows > 1) {
        cv::Rect roi_arrows = cv::Rect(INT_MAX, INT_MAX, INT_MIN, INT_MIN);

        // 1. draw the hadmap and detect
        cv::Mat im_test = cv::Mat::zeros(_roi_out.height, _roi_out.width, CV_8UC1);
        std::vector<std::vector<cv::Point> > contours;

        for (int i = 0; i < sz_arrows_det; ++i) {
            std::vector<cv::Point> contour;

            for (int j = 0; j < ret_info.arrows_det[i].pts_ipm.size(); ++j) {
                contour.push_back(ret_info.arrows_det[i].pts_ipm[j]);
            }

            cv::Rect rt = cv::boundingRect(contour);
            roi_arrows.x = std::min(roi_arrows.x, rt.x);
            roi_arrows.y = std::min(roi_arrows.y, rt.y);
            roi_arrows.width = std::max(roi_arrows.width, rt.x + rt.width);
            roi_arrows.height = std::max(roi_arrows.height, rt.y + rt.height);

            contours.push_back(contour);
            cv::drawContours(im_test, contours, i, cv::Scalar(1), -1);
        }

#ifdef localization_display
        cv::imshow("im_test", im_test * 255);
#endif

        cv::Mat im_template = cv::Mat::zeros(_roi_out.height, _roi_out.width, CV_8UC1);
        contours.clear();

        for (int i = 0; i < sz_arrows; ++i) {
            std::vector<cv::Point> contour;

            for (int j = 0; j < ret_info.arrows[i].pts_ipm.size(); ++j) {
                contour.push_back(ret_info.arrows[i].pts_ipm[j]);
            }

            contours.push_back(contour);
            cv::drawContours(im_template, contours, i, cv::Scalar(1), -1);
        }

#ifdef localization_display
        cv::imshow("im_template", im_template * 255);
#endif
        cv::Mat results;
        cv::matchTemplate(im_template, im_test(cv::Rect(roi_arrows.x, roi_arrows.y,
                        roi_arrows.width - roi_arrows.x, roi_arrows.height - roi_arrows.y)), 
                        results, CV_TM_CCORR);
        double min_val;
        double max_val;
        cv::Point min_pt;
        cv::Point max_pt;
        cv::minMaxLoc(results, &min_val, &max_val, &min_pt, &max_pt);
        double total_val = (roi_arrows.width - roi_arrows.x) * (roi_arrows.height - roi_arrows.y);
    }

    return SYS_VO;
}

#endif
