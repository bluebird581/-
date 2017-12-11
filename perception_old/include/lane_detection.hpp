#ifndef LANE_DETECTION_LANE_H
#define LANE_DETECTION_LANE_H

#ifndef __GNUC__
#include <windows.h>
#undef min
#undef max
#endif

#include <vector>
#include "ipm.hpp"
#include "ransac.hpp"
#include "caffe_cnn.hpp"
#include "basic_struct.h"

enum NEIGHBORS {
    NORMAL = 0,
    CONFLICT = 1,
};

class SLLaneDetection {
public:
    // 1. the main interface
    SLLaneDetection(
        const cv::Rect& roi_in,
        const cv::Rect& roi_out,
        const IPMTransformer& ipm_transformer,
        const cv::Point2f& camera_pos_ipm,
        const std::string& model_path) {
        _roi_in = roi_in;
        _roi_out = roi_out;
        _ipm_transformer = ipm_transformer;
        _camera_pos_ipm = camera_pos_ipm;
        // 1. the main modules
        _ransac_fitting.do_init(0.9999, 0.5, 1);
        _caffe_cnn.do_init(model_path);
        // 2. the size
        _im_w = 1224;
        _im_h = 1024;
        _im_warp_w = _roi_out.width;
        _im_warp_h = _roi_out.height;
    }
    int do_lane_detection(const cv::Mat& im, int lane_num, const std::string& save_name);

    // 2. the sub interface
    IPMTransformer get_transformer() {
        return _ipm_transformer;
    }
    std::vector<LANE> get_lanes() {
        return _lanes;
    }
    cv::Mat get_im_warp() {
        return _im_warp;
    }

private:

    int filter_image_horizonal();
    int generate_lanes_candidate();
    int generate_lanes_scored();
    int generate_lanes_merged();
    int generate_lanes_refined(int lane_num);
    int show_image(const cv::Mat& im, const std::string win_name);

    cv::Mat generate_mask();
    int collect_points(
        const cv::Mat& im,
        const cv::Mat& weights,
        const cv::Rect& roi,
        std::vector<WEIGHTEDPOINT>& pts_line);
    int calculate_average_width(
        const cv::Mat& im,
        const cv::Rect& roi);
    float from_y_to_x(const cv::Vec4f& vec, const float& y, const int& type);
    LANE init_lane_from_pts(
        const std::vector<WEIGHTEDPOINT>& pts_line);

    cv::Mat crop_img_for_lane(
        const cv::Point2f& start_pt,
        const cv::Point2f& end_pt,
        const cv::Point2f& center_pt);
    int merge_two_lanes(LANE& l1, const LANE& l2);
    std::vector<std::vector<NEIGHBORS> > fill_weights();
    int find_best_combo(
        std::vector<LANE>& combo_best,
        std::vector<LANE>& combo_tmp,
        const std::vector<std::vector<NEIGHBORS> >& weights,
        float& score_max,
        int level,
        int statue,
        const int& sz,
        const int& sz_thres);

private:
    // 1. the main module
    RansacFitting _ransac_fitting;
    IPMTransformer _ipm_transformer;
    CaffeCNN _caffe_cnn;

    // 2. the stored ims
    cv::Mat _im_mask;

    cv::Mat _im;
    cv::Mat _im_warp;
    cv::Rect _roi_in;
    cv::Rect _roi_out;
    cv::Mat _im_warp_gray;
    cv::Mat _im_filter;

    // 3. key parameters
    int _im_h;
    int _im_w;
    int _im_warp_h;
    int _im_warp_w;
    cv::Point2f _camera_pos_ipm;
    cv::Point2f _camera_pos_img;
    std::string _save_name;

    // 4. lanes, barriers and ret
    std::vector<LANE> _lanes;
};


/*****************************detailed function*******************************/

int SLLaneDetection::filter_image_horizonal() {
    // 1. from bgr to gray
    cv::cvtColor(_im_warp, _im_warp_gray, CV_BGR2GRAY);
    // 2. filter
    cv::Mat im_smooth2;
    cv::blur(_im_warp_gray, im_smooth2, cv::Size(5, 11));
    _im_filter = cv::Mat::zeros(_im_warp_h, _im_warp_w, CV_32FC1);
    const int w_shift = 5;
    const int w_shift_all = w_shift + (w_shift - 1) / 2;

    for (int r = 0; r < _im_warp_h; ++r) {
        uchar* p_data1 = im_smooth2.ptr<uchar>(r);
        float* p_data2 = _im_filter.ptr<float>(r);

        for (int c = w_shift_all; c < _im_warp_w - w_shift_all; ++c) {
            int middle_v = p_data1[c];
            int left_v = p_data1[c - w_shift];
            int right_v = p_data1[c + w_shift];

            if (middle_v > left_v && middle_v > right_v) {
                p_data2[c] = (middle_v << 1) - left_v - right_v;
            }
        }
    }

    // 3. normalize to [0, 1]
    cv::normalize(_im_filter, _im_filter, 1.0f, 0.0f, CV_MINMAX);
    return 0;
}

int SLLaneDetection::calculate_average_width(
    const cv::Mat& im,
    const cv::Rect& roi) {
    float average_width = 0.0f;
    int count = 0;

    // 1. scan all rows
    for (int r = roi.y; r <= roi.y + roi.height; ++r) {
        const uchar* p_data1 = im.ptr<uchar>(r);
        // 1.1. find start and end x
        int start_x = INT_MAX;
        int end_x = INT_MIN;

        for (int c = roi.x; c <= roi.x + roi.width; ++c) {
            if (p_data1[c] == 128) {
                start_x = c;
                break;
            }
        }

        for (int c = roi.x + roi.width; c >= roi.x; --c) {
            if (p_data1[c] == 128) {
                end_x = c;
                break;
            }
        }

        // 1.2. calculate the width
        if (start_x <= end_x) {
            average_width += (end_x - start_x + 1);
            ++count;
        }
    }

    return average_width / count;
}

int SLLaneDetection::collect_points(
    const cv::Mat& im,
    const cv::Mat& weights,
    const cv::Rect& roi,
    std::vector<WEIGHTEDPOINT>& pts_line) {
    // 1. for each row, only preserve on
    int start_y = std::max(roi.y + roi.height - 60, roi.y);

    for (int r = start_y; r <= roi.y + roi.height; ++r) {
        const uchar* p_data1 = im.ptr<uchar>(r);
        const float* p_data2 = weights.ptr<float>(r);
        // 1.1. calculate the average c*w and w
        float c_w_sum = 0;
        float w_sum = 0;
        int w_count = 0;

        for (int c = roi.x; c <= roi.x + roi.width; ++c) {
            if (p_data1[c] == 128) {
                c_w_sum += c * p_data2[c];
                w_sum += p_data2[c];
                ++w_count;
            }
        }

        // 1.2. append to the vector
        if (w_count > 0) {
            WEIGHTEDPOINT pt;
            pt.pt = cv::Point2f(r, c_w_sum / w_sum);
            pt.weight = w_sum * 255 / w_count;
            pts_line.push_back(pt);
        }
    }

    return 0;
}

float SLLaneDetection::from_y_to_x(const cv::Vec4f& vec, const float& y, const int& type) {
    // 1. since v0 * (y - v2) + v1 *(x - v3) = 0
    return -vec[0] / vec[1] * (y - vec[2]) + vec[3];
}

LANE SLLaneDetection::init_lane_from_pts(
    const std::vector<WEIGHTEDPOINT>& pts_line) {
    // 1. ransac fitting
    cv::Vec4f vec_line;
    std::vector<WEIGHTEDPOINT> ret_pts_line = _ransac_fitting.ransac_fitting_weighted(pts_line,
            vec_line);
    LANE lane;
    lane.id = -1;

    if (ret_pts_line.size() >= 4) {
        // 1.1. prepare the ipm data
        lane.id = 1;
        lane.status = 0;
        lane.vec_ipm = vec_line;
        lane.pts_ipm = ret_pts_line;
        lane.start_pt_ipm = lane.pts_ipm[0].pt;
        lane.end_pt_ipm = lane.pts_ipm[lane.pts_ipm.size() - 1].pt;
        lane.center_pt_ipm = cv::Point2f((lane.start_pt_ipm.x + lane.end_pt_ipm.x) / 2,
                                         (lane.start_pt_ipm.y + lane.end_pt_ipm.y) / 2);
        lane.len_ipm = sqrt((lane.end_pt_ipm.x - lane.start_pt_ipm.x) *
                            (lane.end_pt_ipm.x - lane.start_pt_ipm.x) +
                            (lane.end_pt_ipm.y - lane.start_pt_ipm.y) *
                            (lane.end_pt_ipm.y - lane.start_pt_ipm.y));
        _ipm_transformer.xy_to_rtheta(lane.start_pt_ipm, lane.end_pt_ipm, lane.radium_ipm,
                                      lane.theta_ipm, cv::Point2f(0, _im_warp.rows / 2));
        // 1.2. prepare the img data
        lane.start_pt_img = _ipm_transformer.pt_transformation(lane.start_pt_ipm,
                            _ipm_transformer.get_ipm_to_img());
        lane.end_pt_img = _ipm_transformer.pt_transformation(lane.end_pt_ipm,
                          _ipm_transformer.get_ipm_to_img());
        lane.center_pt_img = cv::Point2f((lane.start_pt_img.x + lane.end_pt_img.x) / 2,
                                         (lane.start_pt_img.y + lane.end_pt_img.y) / 2);
        lane.len_img = sqrt((lane.end_pt_img.x - lane.start_pt_img.x) *
                            (lane.end_pt_img.x - lane.start_pt_img.x) +
                            (lane.end_pt_img.y - lane.start_pt_img.y) *
                            (lane.end_pt_img.y - lane.start_pt_img.y));
        // 1.3. calculate the prob
        float prob = 0.0f;
        int count = 0;

        for (int i = 0; i < ret_pts_line.size(); ++i) {
            prob += ret_pts_line[i].weight;
            ++count;
        }

        lane.conf = prob / count / 255;
    }

    return lane;
}

int SLLaneDetection::generate_lanes_candidate() {
    // 1. threshold and get contours
    cv::Mat im_bw;
    cv::threshold(_im_filter, im_bw, 0.2, 255, CV_THRESH_BINARY);
    im_bw.convertTo(im_bw, CV_8UC1, 1);
    // 2. kick of texts, arrows
    cv::MSER ms(5, 500, 2000);
    std::vector<std::vector<cv::Point> > regions;
    ms(_im_warp_gray, regions, cv::Mat());

    //  {
    //      cv::Mat img_draw = _im_warp.clone();
    //      for (int i = 0; i < regions.size(); i++)
    //      {
    //          cv::RotatedRect rrt = cv::minAreaRect(regions[i]);
    //
    //          int h = std::max(rrt.size.width, rrt.size.height);
    //          int w = std::min(rrt.size.width, rrt.size.height);
    //
    //          if (h > w * 5 || h >= 120 || w >= 50)
    //          {
    //              continue;
    //          }
    //
    //          cv::Scalar color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
    //          for (int j = 0; j < regions[i].size(); ++j)
    //          {
    //              circle(img_draw, regions[i][j], 1, color, 1);
    //          }
    //
    //          ellipse(img_draw, fitEllipse(regions[i]), color);
    //
    //          char texts[1024];
    //#ifdef __GNUC__
    //          snprintf(texts, 1024, "%1.2f", _lanes[i].conf);
    //#else
    //          sprintf_s(texts, "%1.2f", rrt.angle);
    //#endif
    //          cv::putText(img_draw, texts, regions[i][0], cv::FONT_HERSHEY_SIMPLEX,
    //              0.3, cv::Scalar(0, 255, 0), 1);
    //      }
    //      cv::imshow("mser", img_draw);
    //      cv::waitKey(0);
    //  }

    //cv::Mat im_mask = cv::Mat::zeros(_im_h, _im_w, CV_8UC1);
    //cv::Mat im_mask_tmp;

    for (int i = 0; i < regions.size(); ++i) {
        cv::RotatedRect rrt = cv::minAreaRect(regions[i]);
        int h = std::max(rrt.size.width, rrt.size.height);
        int w = std::min(rrt.size.width, rrt.size.height);

        if (h > w * 5 || h >= 120 || w >= 50) {
            continue;
        }

        //im_mask.copyTo(im_mask_tmp);
        //drawContours(im_mask_tmp, regions, i, cv::Scalar(128), CV_FILLED);
        //cv::Rect rt = boundingRect(regions[i]);
        //int h_actual = rt.height;
        //int w_actual = calculate_average_width(im_mask_tmp, rt);
        //if (w_actual <= 20)
        //{
        //  continue;
        //}
        drawContours(im_bw, regions, i, cv::Scalar(0), CV_FILLED);
    }

    //cv::imshow("sb", im_bw);
    cv::Mat im_bw_tmp = im_bw.clone();
    std::vector<std::vector<cv::Point> > contours;
    findContours(im_bw_tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // 2. scan each contour, generate the candidate lanes
    for (int i = 0; i < contours.size(); ++i) {
        // 2.1. filter by angle and length
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

        // 2.2. ransac fitting
        im_bw.copyTo(im_bw_tmp);
        drawContours(im_bw_tmp, contours, i, cv::Scalar(128), CV_FILLED);
        cv::Rect rt = boundingRect(contours[i]);
        std::vector<WEIGHTEDPOINT> pts_line;
        collect_points(im_bw_tmp, _im_filter, rt, pts_line);
        LANE lane = init_lane_from_pts(pts_line);
        lane.len_actual = rt.height;
        lane.sub_num = 1;

        if (lane.len_img < 32 || lane.len_ipm < 12) {
            continue;
        }

        _lanes.push_back(lane);
    }

    return 0;
}

cv::Mat SLLaneDetection::crop_img_for_lane(
    const cv::Point2f& start_pt,
    const cv::Point2f& end_pt,
    const cv::Point2f& center_pt) {
    // 1. determine the size according to the distance
    //int min_x = std::min(start_pt.x, end_pt.x);
    //int min_y = std::min(start_pt.y, end_pt.y);
    //int max_x = std::max(start_pt.x, end_pt.x);
    //int max_y = std::max(start_pt.y, end_pt.y);
    cv::Point2f pt = cv::Point2f(612, 340);
    float dist_x = fabs(center_pt.x - pt.x);
    float dist_y = fabs(center_pt.y - pt.y);
    // bigger dist_y, bigger size; bigger dist_x, smaller size
    //int half_sz = log(dist_y / 2048 + 1) * 500 * exp(-dist_x / 3000);// *1.125;
    int half_sz = log(dist_y / 2048 + 1) * 500 * exp(-dist_x / 3000);
    //int half_sz = std::min(192, int(std::max(
    //  std::min(abs(start_pt.x - center_pt.x), abs(center_pt.x - end_pt.x)),
    //  std::min(abs(start_pt.y - center_pt.y), abs(center_pt.y - end_pt.y)))));
    int border = std::min(fabs(center_pt.x),
                          std::min(fabs(center_pt.y),
                                   std::min(fabs(center_pt.x - _im_w),
                                            fabs(center_pt.y - _im_h))));
    half_sz = std::min(half_sz, border);
    int sz = (half_sz << 1);
    // 2. crop img
    int min_x = center_pt.x - half_sz;
    int min_y = center_pt.y - half_sz;
    //cv::Mat im_draw = _im.clone();
    //cv::rectangle(im_draw, cv::Rect(min_x, min_y, sz, sz), cv::Scalar(0, 255, 0), 2);
    ////cv::line(im_draw, _lanes[i].start_pt_ipm, _lanes[i].end_pt_ipm, cv::Scalar(0, 255, 0), 2);
    //cv::namedWindow("im", 0);
    //cv::imshow("im", im_draw);
    //cv::imshow("ipm", _im(cv::Rect(min_x, min_y, sz, sz)));
    //cv::waitKey(0);
    return _im(cv::Rect(min_x, min_y, sz, sz));
}

int SLLaneDetection::generate_lanes_scored() {
    float alpha = 0.2;// 0.5;

    // 1. scan every lanes
    for (int i = 0; i < _lanes.size(); ++i) {
        // 1.1. crop img and ipm to score
        cv::Mat img = crop_img_for_lane(_lanes[i].start_pt_img, _lanes[i].end_pt_img,
                                        _lanes[i].center_pt_img);
        cv::resize(img, img, cv::Size(48, 48));
        //cv::resize(ipm, ipm, cv::Size(32, 32));
        //continue;
        //cv::Mat composite = cv::Mat::zeros(64, 128, CV_8UC3);
        //cv::resize(img, composite(cv::Rect(0, 0, 64, 64)), cv::Size(64, 64));
        //cv::resize(ipm, composite(cv::Rect(64, 0, 32, 32)), cv::Size(32, 32));
        //cv::imshow("composite", composite);
        //cv::waitKey(0);
        //// 1.2. save images
        //char path_name[1024];
        //sprintf(path_name, "F:\\Project\\adas\\self_localization_lane\\self_localization_lane\\samples\\%s_%f_%f_%f_%f_%f_%f_%f_%f.png",
        //  _save_name.c_str(),
        //  _lanes[i].vec_ipm.val[0],
        //  _lanes[i].vec_ipm.val[1],
        //  _lanes[i].vec_ipm.val[2],
        //  _lanes[i].vec_ipm.val[3],
        //  _lanes[i].start_pt_ipm.x,
        //  _lanes[i].start_pt_ipm.y,
        //  _lanes[i].end_pt_ipm.x,
        //  _lanes[i].end_pt_ipm.y);
        //cv::imwrite(path_name, img);
        //continue;
        // 1.2. use cnn to judge
        std::vector<CNNRECOG> results;
        _caffe_cnn.do_cnn_recog(img, results);
        //_caffe_cnn.do_dvcnn_recog(img, ipm, results);
        //cv::Mat composite = cv::Mat::zeros(128, 256, CV_8UC3);
        //cv::resize(img, composite(cv::Rect(0, 0, 128, 128)), cv::Size(128, 128));
        //cv::resize(ipm, composite(cv::Rect(128, 0, 64, 64)), cv::Size(64, 64));
        //char texts[1024];
        //sprintf_s(texts, "%d:%1.3f", results[0].label, results[0].prob);
        //cv::putText(img, texts, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX,
        //  0.4, cv::Scalar(0, 255, 0), 1);
        //cv::imshow("img", img);
        //cv::waitKey(0);
        //_lanes[i].type = 1;
        _lanes[i].conf = _lanes[i].conf * alpha + results[1].prob * (1 - alpha);

        if (_lanes[i].conf < 0.3) {
            _lanes.erase(_lanes.begin() + i);
            --i;
        }

        //std::cout << "lane" << i << ":" << _lanes[i].type << "-" << _lanes[i].conf << std::endl;
    }

    return 0;
}

bool sort_by_conf_desc(const LANE& l1, const LANE& l2) {
    return l1.conf > l2.conf;
}

bool sort_by_start_asc(const LANE& l1, const LANE& l2) {
    return l1.start_pt_ipm.y < l2.start_pt_ipm.y;
}

bool sort_by_y_asc(const WEIGHTEDPOINT& pt1, const WEIGHTEDPOINT& pt2) {
    return pt1.pt.y < pt2.pt.y;
}

int SLLaneDetection::merge_two_lanes(LANE& l1, const LANE& l2) {
    LANE lane;
    lane.pts_ipm.insert(lane.pts_ipm.end(), l1.pts_ipm.begin(), l1.pts_ipm.end());
    lane.pts_ipm.insert(lane.pts_ipm.end(), l2.pts_ipm.begin(), l2.pts_ipm.end());
    lane.len_ipm = (l1.len_ipm * l1.conf + l2.len_ipm * l2.conf)
                   / (l1.conf + l2.conf);
    lane.conf = l1.conf + l2.conf;
    float weight1 = l1.conf * l1.len_ipm;
    float weight2 = l2.conf * l2.len_ipm;
    lane.theta_ipm = (weight1 * l1.theta_ipm + weight2 * l2.theta_ipm) / (weight1 + weight2);
    lane.radium_ipm = (weight1 * l1.radium_ipm + weight2 * l2.radium_ipm) / (weight1 + weight2);
    lane.sub_num = l1.sub_num + l2.sub_num;
    lane.len_actual = l1.len_actual + l2.len_actual;
    lane.start_pt_ipm = l1.start_pt_ipm.y < l2.start_pt_ipm.y ? l1.start_pt_ipm : l2.start_pt_ipm;
    lane.end_pt_ipm = l1.end_pt_ipm.y > l2.end_pt_ipm.y ? l1.end_pt_ipm : l2.end_pt_ipm;
    l1 = lane;
    return 0;
}

int SLLaneDetection::generate_lanes_merged() {
    // 1. sort lanes according to the confidences
    std::sort(_lanes.begin(), _lanes.end(), sort_by_start_asc);

    // 2. link dash lines
    for (int i = 0; i < int(_lanes.size()) - 1; ++i) {
        for (int j = i + 1; j < _lanes.size(); ++j) {
            // 2.1. if not overlap, skip
            cv::Point2f start_pt = _lanes[i].start_pt_ipm.y > _lanes[j].start_pt_ipm.y ?
                                   _lanes[i].start_pt_ipm : _lanes[j].start_pt_ipm;
            cv::Point2f end_pt = _lanes[i].end_pt_ipm.y < _lanes[j].end_pt_ipm.y ?
                                 _lanes[i].end_pt_ipm : _lanes[j].end_pt_ipm;

            if (start_pt.y < end_pt.y) {
                continue;
            }

            // 2.2. if j can be merged, merge it
            if ((fabs(start_pt.x - end_pt.x) < 5
                    || fabs(_lanes[i].radium_ipm - _lanes[j].radium_ipm) < 5)
                    && fabs(_lanes[i].theta_ipm - _lanes[j].theta_ipm) < 5 * CV_PI / 180)
                //if (abs(_lanes[i].radium_ipm - _lanes[j].radium_ipm) < 10
                //  && abs(_lanes[i].theta_ipm - _lanes[j].theta_ipm) < 5 * CV_PI / 180)
            {
                merge_two_lanes(_lanes[i], _lanes[j]);
                _lanes.erase(_lanes.begin() + j);
                --j;
            }
        }
    }

    // 3. refine dash lines
    for (int i = 0; i < _lanes.size(); ++i) {
        if (_lanes[i].sub_num == 1) {
            continue;
        }

        // 3.1. refine the start pt, end pt, len
        std::sort(_lanes[i].pts_ipm.begin(), _lanes[i].pts_ipm.end(), sort_by_y_asc);

        for (int j = 0; j < _lanes[i].pts_ipm.size(); ++j) {
            std::swap(_lanes[i].pts_ipm[j].pt.x, _lanes[i].pts_ipm[j].pt.y);
        }

        LANE lane = init_lane_from_pts(_lanes[i].pts_ipm);
        lane.sub_num = _lanes[i].sub_num;
        lane.len_actual = _lanes[i].len_actual;
        lane.conf = _lanes[i].conf / _lanes[i].sub_num;
        //lane.type = 1;
        _lanes[i] = lane;
    }

    return 0;
}

bool sort_by_radium_asc(const LANE& l1, const LANE& l2) {
    return l1.radium_ipm < l2.radium_ipm;
}

bool sort_by_dist_asc(const LANE& l1, const LANE& l2) {
    return l1.dist_to_camera < l2.dist_to_camera;
}

std::vector<std::vector<NEIGHBORS> > SLLaneDetection::fill_weights() {
    int sz = _lanes.size();
    // 1. make space
    std::vector<std::vector<NEIGHBORS> > weights(sz);

    for (int i = 0; i < sz; ++i) {
        weights[i].resize(sz);
    }

    // 2. scan lane pair
    for (int i = 0; i < sz; ++i) {
        for (int j = i + 1; j < sz; ++j) {
            // 2.1. if the angle is bigger, or the radium smaller
            if (fabs(_lanes[i].theta_ipm - _lanes[j].theta_ipm) > 15 * CV_PI / 180
                    || fabs(_lanes[i].radium_ipm - _lanes[j].radium_ipm) < 60 - 30) {
                weights[i][j] = weights[j][i] = CONFLICT;
            } else {
                weights[i][j] = weights[j][i] = NORMAL;
            }
        }
    }

    return weights;
}

int SLLaneDetection::find_best_combo(
    std::vector<LANE>& combo_best,
    std::vector<LANE>& combo_tmp,
    const std::vector<std::vector<NEIGHBORS> >& weights,
    float& score_max,
    int level,
    int statue,
    const int& sz,
    const int& sz_thres) {
    // 1. find a better combo
    if (combo_tmp.size() > 0) {
        float score_tmp = 0.0f;

        for (int i = 0; i < combo_tmp.size(); ++i) {
            score_tmp += combo_tmp[i].conf + (combo_tmp[i].len_actual > 180);
        }

        if (score_tmp > score_max) {
            score_max = score_tmp;
            combo_best = combo_tmp;
        }
    }

    // 2. stop the find
    if (combo_tmp.size() >= sz_thres) {
        return 0;
    }

    // 3. go on the find
    for (int i = level + 1; i < sz; ++i) {
        int flag = 0;

        if (level >= 0) {
            flag = weights[level][i];

            if (statue + flag >= 1) {
                continue;
            }
        }

        combo_tmp.push_back(_lanes[i]);
        find_best_combo(combo_best, combo_tmp, weights, score_max, i, statue + flag, sz, sz_thres);
        combo_tmp.pop_back();
    }

    return 0;
}

int SLLaneDetection::generate_lanes_refined(int channel_num) {
    int sz = _lanes.size();

    // 1. calculate the distance to the camera pos?
    for (int i = 0; i < sz; ++i) {
        cv::Vec4f vec;
        vec = _lanes[i].vec_ipm;
        _lanes[i].dist_to_camera = -(vec[0] * (_camera_pos_ipm.y - vec[2]) + vec[1] *
                                     (_camera_pos_ipm.x - vec[3]));
    }

    std::sort(_lanes.begin(), _lanes.end(), sort_by_dist_asc);
    // 2. calculate the weight matrix
    std::vector<std::vector<NEIGHBORS> > weights = fill_weights();
    // 3. find the best combo
    std::vector<LANE> lane_combo_max;
    std::vector<LANE> lane_combo_tmp;
    float score_max = 0;
    find_best_combo(lane_combo_max, lane_combo_tmp, weights, score_max, -1, 0, sz, 6);
    _lanes = lane_combo_max;

    // 4. out of the nearest lane, only solid lane can be accepted

    //// 2. find the left and right border
    //std::vector<LANE> border_max;
    //float score_max = 0.0f;
    //
    //for (int l = 0; l < sz; ++l)
    //{
    //  if (_lanes[l].dist_to_camera > 0)
    //  {
    //      break;
    //  }

    //  for (int r = l + 1; r < sz; ++r)
    //  {
    //      if (_lanes[r].dist_to_camera <= 0)
    //      {
    //          continue;
    //      }

    //      if (fabs(_lanes[l].dist_to_camera) + fabs(_lanes[r].dist_to_camera) > 60 + 20)
    //      {
    //          continue;
    //      }
    //      if (fabs(_lanes[l].dist_to_camera) + fabs(_lanes[r].dist_to_camera) < 60 - 20)
    //      {
    //          continue;
    //      }

    //      float score_angular = exp(-(_lanes[l].theta_ipm - _lanes[r].theta_ipm) / 15.0f);

    //      // calculate the score (conf)
    //      float score_tmp = _lanes[l].conf + (_lanes[l].len_actual > 180)
    //          + _lanes[r].conf + (_lanes[r].len_actual > 180)
    //          + score_angular;

    //      // evaluate the combo
    //      if (score_tmp > score_max)
    //      {
    //          score_max = score_tmp;

    //          border_max.clear();
    //          border_max.push_back(_lanes[l]);
    //          border_max.push_back(_lanes[r]);

    //          if (_lanes[r].len_actual > 180)
    //          {
    //              break;
    //          }
    //      }
    //  }
    //}

    //// 3. if no border, preserve the best left or right
    //if (border_max.empty())
    //{
    //  float score_max = 0.0f;
    //  for (int l = 0; l < sz; ++l)
    //  {
    //      float score_tmp = _lanes[l].conf + (_lanes[l].len_actual > 180);

    //      // evaluate the combo
    //      if (score_tmp > score_max)
    //      {
    //          score_max = score_tmp;

    //          border_max.clear();
    //          border_max.push_back(_lanes[l]);
    //      }
    //  }
    //}

    //_lanes = border_max;

    for (int i = 0; i < _lanes.size(); ++i) {
        cv::Vec4f vec = _lanes[i].vec_ipm;
        _lanes[i].start_pt_ipm = cv::Point2f(from_y_to_x(vec, 0, 2), _roi_out.y);
        _lanes[i].end_pt_ipm = cv::Point2f(from_y_to_x(vec, _roi_out.height, 2), _roi_out.height);
        _lanes[i].start_pt_img = _ipm_transformer.pt_transformation(_lanes[i].start_pt_ipm,
                                 _ipm_transformer.get_ipm_to_img());
        _lanes[i].end_pt_img = _ipm_transformer.pt_transformation(_lanes[i].end_pt_ipm,
                               _ipm_transformer.get_ipm_to_img());
    }

    return 0;
}

int SLLaneDetection::show_image(const cv::Mat& im, const std::string win_name) {
#ifndef __GNUC__

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
            for (int j = 0; j < _lanes[i].pts_ipm.size(); ++j) {
                cv::circle(im_draw, _lanes[i].pts_ipm[j].pt, 3, cv::Scalar(0, 255, 255), 1);
            }

            cv::line(im_draw, _lanes[i].start_pt_ipm, _lanes[i].end_pt_ipm,
                     cv::Scalar(0, 0, 255), 2);
            char texts[1024];
#ifdef __GNUC__
            snprintf(texts, 1024, "%1.2f", _lanes[i].conf);
#else
            sprintf_s(texts, "%d:%1.2f", _lanes[i].len_actual, _lanes[i].conf);
#endif
            cv::Point2f pt;
            pt.x = (_lanes[i].start_pt_ipm.x + _lanes[i].end_pt_ipm.x) / 2;
            pt.y = (_lanes[i].start_pt_ipm.y + _lanes[i].end_pt_ipm.y) / 2;
            cv::putText(im_draw, texts, pt, cv::FONT_HERSHEY_SIMPLEX,
                        0.3, cv::Scalar(0, 255, 0), 1);
        }

        //      char texts[1024];
        //#ifdef __GNUC__
        //      snprintf(texts, 1024, "channel %d = %1.2f", _ret.channel, _ret.delta_x);
        //#else
        //      sprintf_s(texts, "channel %d = %1.3f", _ret.channel, _ret.delta_x);
        //#endif
        //      cv::putText(im_draw, texts, cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX,
        //          1, cv::Scalar(0, 255, 0), 2);
        //
        //#ifdef __GNUC__
        //      snprintf(texts, 1024, "channel_inv %d = %1.2f", _ret.channel_neg, _ret.delta_x);
        //#else
        //      sprintf_s(texts, "channel_inv %d = %1.3f", _ret.channel_neg, _ret.delta_x);
        //#endif
        //      cv::putText(im_draw, texts, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX,
        //          1, cv::Scalar(0, 255, 0), 2);
        cv::imshow(win_name, im_draw);
    }

#endif
    return 0;
}

int SLLaneDetection::do_lane_detection(
    const cv::Mat& im,
    int channel_num,
    const std::string& save_name) {
    // 0. re-init the results
    _save_name = save_name;
    _lanes.clear();
#ifdef __GNUC__
    struct timespec time1 = { 0, 0 };
    struct timespec time2 = { 0, 0 };
    clock_gettime(CLOCK_REALTIME, &time1);
#else
    LARGE_INTEGER t1, t2, tc;
    QueryPerformanceFrequency(&tc);
    QueryPerformanceCounter(&t1);
#endif
    _im = im.clone();
    // 1. ipm to achieve the transformed image
    cv::warpPerspective(_im, _im_warp,
                        _ipm_transformer.get_img_to_ipm(),
                        cv::Size(_roi_out.width, _roi_out.height));
    //cv::imwrite("ipm.jpg", _im_warp);
    //cv::imshow("_im", _im);
    //cv::imshow("_im_warp", _im_warp);
    // 2. filter to achieve the proposals
    filter_image_horizonal();
    //cv::imshow("_im_filter", _im_filter);
#ifdef __GNUC__
    clock_gettime(CLOCK_REALTIME, &time2);
    std::cout << "[info] time for filter: " << (time2.tv_sec - time1.tv_sec) * 1000 +
              (time2.tv_nsec - time1.tv_nsec) / 1000000 << "ms" << std::endl;
    clock_gettime(CLOCK_REALTIME, &time1);
#else
    QueryPerformanceCounter(&t2);
    std::cout << "[info] time for filter: " << (t2.QuadPart - t1.QuadPart) * 1000.0 / tc.QuadPart
              << std::endl;
    QueryPerformanceCounter(&t1);
#endif
    generate_lanes_candidate();
    show_image(_im_filter, "coarse");
#ifdef __GNUC__
    clock_gettime(CLOCK_REALTIME, &time2);
    std::cout << "[info] time for fitting: " << (time2.tv_sec - time1.tv_sec) * 1000 +
              (time2.tv_nsec - time1.tv_nsec) / 1000000 << "ms" << std::endl;
    clock_gettime(CLOCK_REALTIME, &time1);
#else
    QueryPerformanceCounter(&t2);
    std::cout << "[info] time for fitting: " << (t2.QuadPart - t1.QuadPart) * 1000.0 / tc.QuadPart
              << std::endl;
    QueryPerformanceCounter(&t1);
#endif
    // 3. use cnn to score them
    generate_lanes_scored();
    show_image(_im_warp, "scored");
    //return 0;
#ifdef __GNUC__
    clock_gettime(CLOCK_REALTIME, &time2);
    std::cout << "[info] time for CNN: " << (time2.tv_sec - time1.tv_sec) * 1000 +
              (time2.tv_nsec - time1.tv_nsec) / 1000000 << "ms" << std::endl;
    clock_gettime(CLOCK_REALTIME, &time1);
#else
    QueryPerformanceCounter(&t2);
    std::cout << "[info] time for CNN: " << (t2.QuadPart - t1.QuadPart) * 1000.0 / tc.QuadPart
              << std::endl;
    QueryPerformanceCounter(&t1);
#endif
    // 4. merge the dash lanes
    generate_lanes_merged();
    show_image(_im_warp, "merged");
    // 5. use topology to refine lanes
    generate_lanes_refined(channel_num);
    show_image(_im_warp, "refined");
#ifdef __GNUC__
    clock_gettime(CLOCK_REALTIME, &time2);
    std::cout << "[info] time for refinement: " << (time2.tv_sec - time1.tv_sec) * 1000 +
              (time2.tv_nsec - time1.tv_nsec) / 1000000 << "ms" << std::endl;
    clock_gettime(CLOCK_REALTIME, &time1);
#else
    QueryPerformanceCounter(&t2);
    std::cout << "[info] time for refinement: " << (t2.QuadPart - t1.QuadPart) * 1000.0
              / tc.QuadPart << std::endl;
    QueryPerformanceCounter(&t1);
#endif
    return 0;
}

#endif