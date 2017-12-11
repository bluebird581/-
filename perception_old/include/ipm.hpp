#ifndef LANE_DETECTION_IPM_H
#define LANE_DETECTION_IPM_H

#include <opencv2/opencv.hpp>
#include "basic_struct.h"

class IPMTransformer {
public:

    cv::Point2f pt_transformation(const cv::Point2f& src_pt,
                                  const cv::Mat& trans) {
        cv::Point2f dst_pt;
        cv::Mat pt_m = (cv::Mat_<float>(3, 1) << src_pt.x, src_pt.y, 1);
        cv::Mat pt_m_new = trans * pt_m;
        dst_pt.x = pt_m_new.at<float>(0, 0) / pt_m_new.at<float>(2, 0);
        dst_pt.y = pt_m_new.at<float>(1, 0) / pt_m_new.at<float>(2, 0);

        return dst_pt;
    }

    int do_transformation(const CAMERAINFO& camera_info,
                          const cv::Rect& roi_in,
                          const cv::Rect& roi_out) {
        // 1. prepare the matrix
        float c1 = cos(camera_info.pitch * CV_PI / 180);
        float s1 = sin(camera_info.pitch * CV_PI / 180);
        cv::Mat pitch_m = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, c1, s1, 0, -s1, c1);

        float c2 = cos(camera_info.yaw * CV_PI / 180);
        float s2 = sin(camera_info.yaw * CV_PI / 180);
        cv::Mat yaw_m = (cv::Mat_<float>(3, 3) << c2, s2, 0, -s2, c2, 0, 0, 0, 1);

        float c3 = cos(camera_info.roll * CV_PI / 180);
        float s3 = sin(camera_info.roll * CV_PI / 180);
        cv::Mat roll_m = (cv::Mat_<float>(3, 3) << c3, 0, -s3, 0, 1, 0, s3, 0, c3);

        cv::Mat image_to_camera = (cv::Mat_<float>(3, 3) << 1 / camera_info.fx, 0,
                                   -camera_info.cx / camera_info.fx,
                                   0, 1 / camera_info.fy, -camera_info.cy / camera_info.fy,
                                   0, 0, 1);
        cv::Mat camera_to_world = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 0, -1, 0, 1, 0);

        cv::Mat image_to_world = roll_m * yaw_m * pitch_m * camera_to_world * image_to_camera;
        _img_to_world = image_to_world;
        _ipm_modi_pitch = roll_m * yaw_m * pitch_m;
        // 2. calculate the transformation

        _src_pts[0] = cv::Point2f(roi_in.x, roi_in.y);
        _src_pts[1] = cv::Point2f(roi_in.width, roi_in.y);
        _src_pts[2] = cv::Point2f(roi_in.width, roi_in.height);
        _src_pts[3] = cv::Point2f(roi_in.x, roi_in.height);

        //_src_pts[0] = cv::Point2f(roi_in.x, 0);
        //_src_pts[1] = cv::Point2f(roi_in.width, 0);
        //_src_pts[2] = cv::Point2f(roi_in.width, roi_in.height - roi_in.y);
        //_src_pts[3] = cv::Point2f(roi_in.x, roi_in.height - roi_in.y);

        float min_x = INT_MAX;
        float max_x = INT_MIN;
        float min_y = INT_MAX;
        float max_y = INT_MIN;

        for (int i = 0; i < 4; ++i) {
            _dst_pts[i] = pt_transformation(_src_pts[i], image_to_world);

            min_x = std::min(min_x, _dst_pts[i].x);
            max_x = std::max(max_x, _dst_pts[i].x);
            min_y = std::min(min_y, _dst_pts[i].y);
            max_y = std::max(max_y, _dst_pts[i].y);
        }

        float scale_x = roi_out.width / (max_x - min_x);
        float scale_y = roi_out.height / (max_y - min_y);
        cv::Mat scale_m = (cv::Mat_<float>(3, 3) << scale_x, 0, -min_x * scale_x,
                           0, scale_y, -min_y * scale_y,
                           0, 0, 1);

        //cv::Mat shift_y = (cv::Mat_<float>(3, 3) << 1, 0, 0,
        //  0, 1, roi_in.y,
        //  0, 0, 1);
        _img_to_ipm = scale_m * image_to_world/* * shift_y*/;

        // 3. prepare the left and right part
        //_left_part = scale_m * roll_m * yaw_m;
        //std::cout << "ssss" << _left_part << std::endl;
        //cv::invert(_left_part, _left_part);
        //std::cout << "ssss" << _left_part << std::endl;
        //_right_part = pitch_m * camera_to_world * image_to_camera;

        //for (int i = 0; i < 4; ++i)
        //{
        //  dst_pts[i].x = (dst_pts[i].x - min_x) / (max_x - min_x) * roi_out.width;
        //  dst_pts[i].y = (dst_pts[i].y - min_y) / (max_y - min_y) * roi_out.height;
        //}

        //_img_to_ipm = cv::getPerspectiveTransform(src_pts, dst_pts);
        _img_to_ipm.convertTo(_img_to_ipm, CV_32F);
        cv::invert(_img_to_ipm, _ipm_to_img);

        _width_versus_height = scale_x / scale_y;

        //std::cout << (roi_out.width * (max_y - min_y)) / (roi_out.height * (max_x - min_x)) << std::endl;
        return 0;
    }

public:
    //0912 modify pitch
    cv::Mat _ipm_modi_pitch;

    cv::Mat get_img_to_ipm() {
        return _img_to_ipm;
    }
    cv::Mat get_ipm_to_img() {
        return _ipm_to_img;
    }
    //cv::Mat get_left_part()
    //{
    //  return _left_part;
    //}
    //cv::Mat get_right_part()
    //{
    //  return _right_part;
    //}
    float get_width_versus_height() {
        return _width_versus_height;
    }
    //cv::Point2f* get_src_pts()
    //{
    //  return _src_pts;
    //}
    //cv::Point2f* get_dst_pts()
    //{
    //  return _dst_pts;
    //}

    int xy_to_rtheta(cv::Point2f start_pt,
                     cv::Point2f end_pt,
                     float& radium,
                     float& theta,
                     const cv::Point2f& origin_pt) {
        start_pt -= origin_pt;
        end_pt -= origin_pt;

        if (start_pt.x == end_pt.x) {
            radium = fabs(start_pt.x);
            theta = start_pt.x >= 0 ? 0 : CV_PI;
        } else if (start_pt.y == end_pt.y) {
            radium = fabs(start_pt.y);
            theta = start_pt.y >= 0 ? CV_PI / 2 : -CV_PI / 2;
        } else {
            float r1 = 0;
            float r2 = 0;

            theta = std::atan2(end_pt.x - start_pt.x, start_pt.y - end_pt.y);
            r1 = start_pt.x * std::cos(theta) + start_pt.y * std::sin(theta);
            r2 = end_pt.x * std::cos(theta) + end_pt.y * std::sin(theta);

            if (r1 < 0 || r2 < 0) {
                theta += CV_PI;
                theta -= theta > CV_PI ? 2 * CV_PI : 0;
            }

            radium = fabs(r2);
        }

        return 0;
    }

private:
    cv::Mat _img_to_ipm;
    cv::Mat _ipm_to_img;

    cv::Mat _img_to_world;
    //cv::Mat _left_part;
    //cv::Mat _right_part;
    float _width_versus_height;

    cv::Point2f _src_pts[4];
    cv::Point2f _dst_pts[4];
};

#endif
