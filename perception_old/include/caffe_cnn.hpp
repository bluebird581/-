#ifndef LOCALIZATION_CAFFE_CNN_H
#define LOCALIZATION_CAFFE_CNN_H

#include <opencv2/opencv.hpp>
#include "caffe/caffe.hpp"
#define USE_OPENCV
#ifdef __GNUC__
#include "caffe/layers/memory_data_layer.hpp"
#endif
#include <fstream>

class CaffeCNN {
public:
    CaffeCNN() {
        _caffe_inference = NULL;
    }
    ~CaffeCNN() {
        if (NULL != _caffe_inference) {
            delete _caffe_inference;
            _caffe_inference = NULL;
        }
    }

public:
    int do_init(const std::string& cnn_model,
                const std::string& cnn_prototxt);
    int do_cnn_segmentation(const cv::Mat& im,
                            cv::Mat& mask,
                            std::vector<cv::Mat>& probs);
    //int do_cnn_recog(const cv::Mat &im, std::vector<CNNRECOG> &results);
    //int do_cnn_recog_sorted(const cv::Mat &im, std::vector<CNNRECOG> &results);
    //int do_dvcnn_recog(const cv::Mat &im1, const cv::Mat &im2, std::vector<CNNRECOG> &results);
    //int do_dvcnn_recog_sorted(const cv::Mat &im1, const cv::Mat &im2, std::vector<CNNRECOG> &results);

private:
    caffe::Net<float>* _caffe_inference;
    //int predict(std::vector<CNNRECOG> &results);
};

//bool compare_by_score_desc(const CNNRECOG recog1, const CNNRECOG recog2)
//{
//     return recog1.prob > recog2.prob;
//}

int CaffeCNN::do_init(const std::string& cnn_model,
                      const std::string& cnn_prototxt) {
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    _caffe_inference = new caffe::Net<float>(cnn_prototxt, caffe::TEST);
    _caffe_inference->CopyTrainedLayersFrom(cnn_model);

    if (!_caffe_inference) {
        return -1;
    }

    return 0;
}

int CaffeCNN::do_cnn_segmentation(const cv::Mat& im, cv::Mat& mask, std::vector<cv::Mat>& probs) {
    const boost::shared_ptr<caffe::MemoryDataLayer<float> > memory_data_layer
        = boost::static_pointer_cast<caffe::MemoryDataLayer<float> >
          (_caffe_inference->layer_by_name("data"));

    std::vector<cv::Mat> images(1, im);
    std::vector<int> labels(1, 0);
    memory_data_layer->AddMatVector(images, labels);

    float loss = 0.0f;
    const std::vector<caffe::Blob<float>*>& outs = _caffe_inference->ForwardPrefilled(&loss);
    //const boost::shared_ptr<caffe::Blob<float> >& im_blobs = _caffe_inference->blob_by_name("probs");

    if (outs.size() < 2) {
        return -1;
    }

    int num = outs[1]->num();
    int channels = outs[1]->channels();
    int height = outs[1]->height();
    int width = outs[1]->width();

    if (num != 1) {
        return -1;
    }

    //     std::cout << "num = " << num << std::endl;
    //     std::cout << "channels = " << channels << std::endl;
    //     std::cout << "height = " << height << std::endl;
    //     std::cout << "width = " << width << std::endl;

    //// fill the mask
    //mask = cv::Mat::zeros(im.rows, im.cols, CV_8UC1);
    //for (int i = 0; i < outs[0]->num(); ++i) {
    //     for (int j = 0; j < outs[0]->channels(); ++j) {
    //          const float *p_data = outs[0]->cpu_data();
    //          for (int h = 0; h < outs[0]->height(); ++h) {
    //               for (int w = 0; w < outs[0]->width(); ++w) {
    //                    mask.at<uchar>(h, w) = p_data[h * outs[0]->width() + w];
    //               }
    //          }
    //     }
    //}

    for (int i = 0; i < num; ++i) {
        mask = cv::Mat::zeros(height, width, CV_8U);
        cv::Mat max_val = cv::Mat::zeros(height, width, CV_32F);

        //const float *p_data = outs[1]->cpu_data();
        float* p_data_ori = new float[channels * height * width];

        if (p_data_ori == NULL) {
            std::cout << "memcpy memery is not enough" << std::endl;
            return -1;
        }

        memcpy(p_data_ori, outs[1]->cpu_data(), channels * height * width * sizeof(float));
        float* p_data = p_data_ori;

        for (int j = 0; j < channels; ++j) {
            cv::Mat prob = cv::Mat::zeros(im.rows, im.cols, CV_8U);

            uchar* p_mask = mask.ptr<uchar>(0);
            float* p_max_val = max_val.ptr<float>(0);
            uchar* p_prob = prob.ptr<uchar>(0);

            for (int h = 0; h < height; ++h) {
                for (int w = 0; w < width; ++w) {
                    *p_prob = *p_data * 255;

                    if (*p_data > *p_max_val) {
                        *p_mask = j;
                        *p_max_val = *p_data;
                    }

                    ++p_data;
                    ++p_mask;
                    ++p_max_val;
                    ++p_prob;
                }
            }

            probs.push_back(prob);
        }

        delete p_data_ori;
    }

    return 0;
}

//int CaffeCNN::do_cnn_recog(const cv::Mat &im, std::vector<CNNRECOG> &results)
//{
//     results.clear();
//     if (NULL == _p_caffe_net)
//     {
//          return -1;
//     }
//     const boost::shared_ptr<caffe::MemoryDataLayer<float> > memory_data_layer
//          = boost::static_pointer_cast<caffe::MemoryDataLayer<float> >
//          (_p_caffe_net->layer_by_name("data_2"));
//     //cv::Mat normal;
//     //cv::resize(im, normal, cv::Size(memory_data_layer->width(),
//     //     memory_data_layer->height()));
//     std::vector<cv::Mat> images(1, im);
//     std::vector<int> labels(1, 0);
//     memory_data_layer->AddMatVector(images, labels);
//
//     int ret = predict(results);
//     if (ret != 0) {
//          return ret;
//     }
//
//     return 0;
//}
//
//int CaffeCNN::do_dvcnn_recog(const cv::Mat &im1, const cv::Mat &im2, std::vector<CNNRECOG> &results)
//{
//     results.clear();
//     if (NULL == _p_caffe_net)
//     {
//          return -1;
//     }
//
//     const boost::shared_ptr<caffe::MemoryDataLayer<float> > memory_data_layer1
//          = boost::static_pointer_cast<caffe::MemoryDataLayer<float> >
//          (_p_caffe_net->layer_by_name("data_1"));
//     const boost::shared_ptr<caffe::MemoryDataLayer<float> > memory_data_layer2
//          = boost::static_pointer_cast<caffe::MemoryDataLayer<float> >
//          (_p_caffe_net->layer_by_name("data_2"));
//     //cv::Mat normal1;
//     //cv::resize(im1, normal1, cv::Size(memory_data_layer1->width(),
//     //     memory_data_layer1->height()));
//     //cv::Mat normal2;
//     //cv::resize(im2, normal2, cv::Size(memory_data_layer2->width(),
//     //     memory_data_layer2->height()));
//     std::vector<cv::Mat> image1(1, im1);
//     std::vector<int> label1(1, 0);
//     std::vector<cv::Mat> image2(1, im2);
//     std::vector<int> label2(1, 0);
//     memory_data_layer1->AddMatVector(image1, label1);
//     memory_data_layer2->AddMatVector(image2, label2);
//
//     int ret = predict(results);
//     if (ret != 0) {
//          return ret;
//     }
//
//     return 0;
//}
//
//int CaffeCNN::do_cnn_recog_sorted(const cv::Mat &im, std::vector<CNNRECOG> &results)
//{
//     int ret = do_cnn_recog(im, results);
//     sort(results.begin(), results.end(), compare_by_score_desc);
//
//     return ret;
//}
//
//int CaffeCNN::do_dvcnn_recog_sorted(const cv::Mat &im1, const cv::Mat &im2, std::vector<CNNRECOG> &results)
//{
//     int ret = do_dvcnn_recog(im1, im2, results);
//     sort(results.begin(), results.end(), compare_by_score_desc);
//
//     return ret;
//}

//int CaffeCNN::predict(std::vector<CNNRECOG> &results)
//{
//     results.clear();
//
//     if (NULL == _p_caffe_net) {
//          return -1;
//     }
//     float loss = 0.0f;
//     const std::vector<caffe::Blob<float>*>& outs = _p_caffe_net->ForwardPrefilled(&loss);
//     if (outs.size() < 2) {
//          std::cout << "+   [ERROR]: model profile error." << std::endl;
//          return -1;
//     }
//     int blob_num = static_cast<int>(outs.size());
//     int dim_features = outs[blob_num - 1]->count() / outs[blob_num - 1]->num();
//     for (int j = 0; j < dim_features; ++j) {
//          float prob = outs[blob_num - 1]->cpu_data()[j];
//
//          CNNRECOG result;
//          result.label = j;
//          result.prob = prob;
//          results.push_back(result);
//     }
//
//     return 0;
//}
#endif
