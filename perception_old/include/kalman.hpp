#ifndef LOCALIZATION_KALMAN_H
#define LOCALIZATION_KALMAN_H
#include <opencv2/opencv.hpp>

class Kalman {

public:

    void init(
        const int& measure_num,
        const int& state_num,
        const std::vector<float> measure_vals,
        double process_noise,
        double measure_noise) {
        _measure_num = measure_num;
        _state_num = state_num;

        _kalman.init(_state_num, _measure_num, 0);
        _kalman.transitionMatrix = cv::Mat::eye(_state_num, _state_num, CV_32FC1);

        setIdentity(_kalman.measurementMatrix, cv::Scalar::all(1));
        setIdentity(_kalman.processNoiseCov, cv::Scalar::all(process_noise));
        setIdentity(_kalman.measurementNoiseCov, cv::Scalar::all(measure_noise));
        setIdentity(_kalman.errorCovPost, cv::Scalar::all(1));

        for (int i = 0; i < measure_num; ++i) {
            _kalman.statePost.at<float>(i, 0) = measure_vals[i];
        }

        //_state_m = cv::Mat::zeros(_state_num, 1, CV_32FC1);
        _measure_m = cv::Mat::zeros(_measure_num, 1, CV_32FC1);
    }

    std::vector<float> do_predict() {
        std::vector<float> ret;

        for (int i = 0; i < _state_num; ++i) {
            ret.push_back(_kalman.statePost.at<float>(i, 0));
        }

        return ret;
    }

    int do_update(const std::vector<float> measure_vals) {
        for (int i = 0; i < _measure_num; ++i) {
            _measure_m.at<float>(i, 0) = measure_vals[i];
        }

        _kalman.predict();
        _kalman.correct(_measure_m);

        return 0;
    }

private:
    cv::KalmanFilter _kalman;

    int _state_num;
    int _measure_num;
    //cv::Mat _state_m;
    cv::Mat _measure_m;
};

class Kalman2 {

public:

    void init(
        const int& measure_num,
        const int& state_num,
        const std::vector<float> measure_vals,
        double process_noise,
        double measure_noise) {
        _measure_num = measure_num;
        _state_num = state_num;

        _kalman.init(_state_num, _measure_num, 0);
        _kalman.transitionMatrix = cv::Mat::eye(_state_num, _state_num, CV_32FC1);

        //for (int i = 0; i < _state_num; i += 3)
        //{
        //    _kalman.transitionMatrix.at<float>(i, i + 1) = 1;
        //    _kalman.transitionMatrix.at<float>(i, i + 2) = 0.5;
        //    _kalman.transitionMatrix.at<float>(i + 1, i + 2) = 1;
        //}
        for (int i = 0; i < _state_num; i += 2) {
            _kalman.transitionMatrix.at<float>(i, i + 1) = 1;
        }

        //std::cout << _kalman.transitionMatrix << std::endl;

        setIdentity(_kalman.measurementMatrix, cv::Scalar::all(1));
        setIdentity(_kalman.processNoiseCov, cv::Scalar::all(process_noise));
        setIdentity(_kalman.measurementNoiseCov, cv::Scalar::all(measure_noise));
        setIdentity(_kalman.errorCovPost, cv::Scalar::all(1));

        for (int i = 0; i < measure_num; ++i) {
            _kalman.statePost.at<float>(i * 2, 0) = measure_vals[i];
            _kalman.statePost.at<float>(i * 2 + 1, 0) = measure_vals[i];
            //_kalman.statePost.at<float>(i * 3, 0) = measure_vals[i];
            //_kalman.statePost.at<float>(i * 3 + 1, 0) = measure_vals[i];
            //_kalman.statePost.at<float>(i * 3 + 2, 0) = 0;
        }

        //_state_m = cv::Mat::zeros(_state_num, 1, CV_32FC1);
        _measure_m = cv::Mat::zeros(_measure_num, 1, CV_32FC1);
    }

    std::vector<float> do_predict() {
        std::vector<float> ret;

        for (int i = 0; i < _state_num; ++i) {
            ret.push_back(_kalman.statePost.at<float>(i, 0));
        }

        return ret;
    }

    int do_update(const std::vector<float> measure_vals) {
        for (int i = 0; i < _measure_num; ++i) {
            _measure_m.at<float>(i, 0) = measure_vals[i];
        }

        _kalman.predict();
        _kalman.correct(_measure_m);

        return 0;
    }

private:
    cv::KalmanFilter _kalman;

    int _state_num;
    int _measure_num;
    //cv::Mat _state_m;
    cv::Mat _measure_m;
};

#endif