#ifndef LOCALIZATION_GROUND_SLAM_H
#define LOCALIZATION_GROUND_SLAM_H

#include "basic_struct.h"

class GroundSlam {
public:
    GroundSlam() {
        _r_f = cv::Mat::eye(3, 3, CV_64F);
        _t_f = cv::Mat::zeros(3, 1, CV_64F);
        _r = cv::Mat::eye(3, 3, CV_64F);
        _t = cv::Mat::zeros(3, 1, CV_64F);

        //cv::namedWindow("slam");
        //_im_draw = cv::Mat::zeros(600, 600, CV_8UC3);
    }

public:
    int do_initial_slam();
    SYSTEM_STATUS do_ground_slam(
        const cv::Mat& img,
        const double& scale);
    cv::Point2d get_position() {
        return _pos;
    }

private:
    int slam_by_registration(const double& scale);
    int slam_by_kalmen_filter();

    void feature_tracking(cv::Mat img_1,
                          cv::Mat img_2,
                          std::vector<cv::Point2f>& points1,
                          std::vector<cv::Point2f>& points2,
                          std::vector<uchar>& status);
    void feature_detection(cv::Mat img_1, std::vector<cv::Point2f>& points1);

private:
    std::deque<cv::Mat> _imgs;
    cv::Point2d _pos;

    std::vector<cv::Point2f> _feature_prev;
    std::vector<cv::Point2f> _feature_curr;
    cv::Mat _img_prev;
    cv::Mat _img_curr;

    cv::Mat _r_f;
    cv::Mat _t_f;

    cv::Mat _r;
    cv::Mat _t;

    int _last_interval;

    //cv::Mat _im_draw;
};

void GroundSlam::feature_tracking(cv::Mat img_1, cv::Mat img_2,
                                  std::vector<cv::Point2f>& points1,
                                  std::vector<cv::Point2f>& points2, std::vector<uchar>& status) {

    //this function automatically gets rid of points for which tracking fails

    std::vector<float> err;
    cv::Size win_size = cv::Size(21, 21);
    cv::TermCriteria termcrit =
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err,
                             win_size, 3, termcrit, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int index_correction = 0;

    for (int i = 0; i < status.size(); i++) {
        cv::Point2f pt = points2.at(i - index_correction);

        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
            if ((pt.x < 0) || (pt.y < 0)) {
                status.at(i) = 0;
            }

            points1.erase(points1.begin() + (i - index_correction));
            points2.erase(points2.begin() + (i - index_correction));
            index_correction++;
        }

    }

}

void GroundSlam::feature_detection(cv::Mat img_1, std::vector<cv::Point2f>& points1) {
    std::vector<cv::KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmax_suppression = true;
    cv::FAST(img_1, keypoints_1, fast_threshold, nonmax_suppression);
    cv::KeyPoint::convert(keypoints_1, points1, std::vector<int>());
}

int GroundSlam::slam_by_registration(const double& scale) {
#ifdef USE_SLAM

    if (_feature_prev.size() == 0) {
        feature_detection(_img_curr, _feature_curr);
        _feature_prev = _feature_curr;
        _img_prev = _img_curr.clone();
        return -1;
    }

    // 1. track the feature
    std::vector<uchar> status;
    feature_tracking(_img_prev, _img_curr, _feature_prev, _feature_curr, status);

    //cv::Mat feature1 = _img_prev.clone();
    //cv::Mat feature2 = _img_curr.clone();
    //for (int i = 0; i < _feature_prev.size(); ++i)
    //{
    //    cv::circle(feature1, _feature_prev[i], 2, cv::Scalar(0, 0, 255), -1);
    //}
    //for (int i = 0; i < _feature_curr.size(); ++i)
    //{
    //    cv::circle(feature2, _feature_curr[i], 2, cv::Scalar(0, 255, 0), -1);
    //}

    //imshow("sb1", feature1);
    //imshow("sb2", feature2);

    // 2. calculate the E matrix
    cv::Mat e;
    cv::Mat r;
    cv::Mat t;
    cv::Mat mask;
    cv::Point2f pp = cv::Point2f(612, 512);
    e = cv::findEssentialMat(_feature_curr, _feature_prev, 700, pp, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(e, _feature_curr, _feature_prev, r, t, 700, pp, mask);

    bool flag = false;

    // 3. update the pos
    //if ((t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

    cv::Mat delta = scale * (_r_f * t);

    //std::cout << t;
    //std::cout << delta;

    if (fabs(delta.at<double>(2)) > 200 || fabs(delta.at<double>(0)) > 200
            || (t.at<double>(2) < t.at<double>(0)) || (t.at<double>(2) < t.at<double>(1))) {
        _t_f = _t_f + scale * (_r_f * _t);
        _r_f = _r * _r_f;
    } else {
        _t_f = _t_f + scale * (_r_f * t);
        _r_f = R * _r_f;
        _r = R.clone();
        _t = t.clone();
    }

    int x = int(_t_f.at<double>(0) / 50) + 300;
    int y = int(_t_f.at<double>(2) / 50) + 100;

    //std::cout << _t_f << std::endl;
    //cv::circle(_im_draw, cv::Point(x, y), 1, CV_rGB(255, 0, 0), 2);
    //cv::imshow("slam", _im_draw);

    //}

    feature_detection(_img_prev, _feature_prev);
    feature_tracking(_img_prev, _img_curr, _feature_prev, _feature_curr, status);

    _feature_prev = _feature_curr;
    _img_prev = _img_curr.clone();


    //int x = int(_t_f.at<double>(0)) + 300;
    //int y = int(_t_f.at<double>(2)) + 100;
    //cv::circle(_im_draw, cv::Point(x, y), 1, CV_rGB(255, 0, 0), 2);

    //cv::imshow("slam", _im_draw);
    _pos = cv::Point2d(_t_f.at<double>(0), _t_f.at<double>(2));

#else
    _pos = cv::Point2d(-1000000, -1000000);

#endif

    return 0;
}

int GroundSlam::do_initial_slam() {
    // 1. initial the R, T
    _r_f = cv::Mat::eye(3, 3, CV_64F);
    _r = cv::Mat::eye(3, 3, CV_64F);
    _t_f = cv::Mat::zeros(3, 1, CV_64F);
    _t = cv::Mat::zeros(3, 1, CV_64F);

    _last_interval = 0;

    return 0;
}

SYSTEM_STATUS GroundSlam::do_ground_slam(
    const cv::Mat& img,
    const double& scale) {
    _img_curr = img.clone();
    _feature_curr.clear();

    slam_by_registration(scale);

    return SYS_SUCCESS_Y;
}

#endif