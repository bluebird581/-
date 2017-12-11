#ifndef SCENE_PARSE_H
#define SCENE_PARSE_H

#include "basic_struct.h"
#include "caffe_cnn.hpp"
#include "timer.hpp"

class SceneParse {
public:
    int do_init(
        const std::string& cnn_model,
        const std::string& cnn_prototxt) {
        int flag = _caffe_cnn.do_init(cnn_model, cnn_prototxt);

//         count = 0;
        return flag;
    }

    SYSTEM_STATUS do_scene_parse(
        const cv::Mat& ipm_ori,
        cv::Mat& ipm_mask,
        std::vector<cv::Mat>& ipm_probs) {
        //fk apolo std::cout << "    [-] Begin Scene Parsing" << std::endl;
        _timer.do_start_timer();

        ipm_probs.clear();
        int flag = _caffe_cnn.do_cnn_segmentation(ipm_ori, ipm_mask, ipm_probs);

//         char nam[100];
//         sprintf(nam, "image/mask_%d.jpg", ++count);
//         cv::imwrite(nam, ipm_mask * 50);
        //cv::imshow("ori", ipm_ori);
        //cv::imshow("seg", ipm_mask * 50);

        //fk apolo        _timer.do_end_timer("    [-] End Scene Parsing, misc time: ");

        if (!flag) {
            return SYS_GROUND_DETECT;
        } else {
            return SYS_ERR;
        }
    }

private:
    CaffeCNN _caffe_cnn;
    Timer _timer;
//     int count;
};

#endif
