#ifndef LOCALIZATION_HADMAP_DATA_WEB_H
#define LOCALIZATION_HADMAP_DATA_WEB_H

#include <curl/curl.h>
#include <opencv2/opencv.hpp>
#include <set>
//#ifdef __GNUC__
//#include "json/json.h"
//#else
//#include "json.h"
//#endif
#include "basic_struct.h"

typedef struct TASK1 {
    std::string id;
    int image_id;
    int image_idx;
    int task_id;
    cv::Vec3d xyzpoint;

};

class HadmapDataWeb {

public:
    HadmapDataWeb();
    ~HadmapDataWeb();

    cv::Vec3d lonlat_to_mercator(const cv::Vec3d& lonlat) {
        cv::Vec3d mercator;

        mercator[0] = lonlat[0] * 20037508.34 / 180;
        mercator[1] = log(tan((90 + lonlat[1]) * CV_PI / 360)) / (CV_PI / 180);
        mercator[1] = mercator[1] * 20037508.34 / 180;

        return mercator;
    }

    int do_request_task1(std::string url_name,
                         std::string pic_id,
                         std::vector<TASK1>& task1_return_data);
private:

    CURL* _curl;

};

HadmapDataWeb::HadmapDataWeb() {
    _curl = curl_easy_init();

    if (!_curl) {
        std::cout << "[error] could not initialize the guoke communication model" << std::endl;
    }
}

HadmapDataWeb::~HadmapDataWeb() {
    curl_easy_cleanup(_curl);
}


static size_t guoke_request_callback(void* ptr, size_t size, size_t nmemb, std::string* stream) {
    stream->append((char*)ptr, size * nmemb);
    stream->append("\0");

    return size * nmemb;
}

int HadmapDataWeb::do_request_task1(std::string url_name, std::string pic_id,
                                    std::vector<TASK1>& task1_return_data) {
    //std::cout << url_name << std::endl;
    CURLcode res;
    char json_data[1024];
    memset(json_data, 0, sizeof(json_data));
    std::string json_str = "{";
    json_str += "\"pic_id\" : \"";
    json_str += pic_id;
    json_str += "\"";
    json_str += "}";
    std::string str_response;

#ifdef __GNUC__
    strcpy(json_data, json_str.c_str());
#else
    strcpy_s(json_data, json_str.c_str());
#endif

    curl_easy_setopt(_curl, CURLOPT_TIMEOUT, 1);
    curl_easy_setopt(_curl, CURLOPT_URL, url_name.c_str());
    curl_slist* ptr_curl_slist = curl_slist_append(NULL,
                                 "Content-Type:application/json;charset=UTF-8");
    curl_easy_setopt(_curl, CURLOPT_HTTPHEADER, ptr_curl_slist);
    //curl_easy_setopt(_curl, CURLOPT_POSTFIELDS, json_data);
    curl_easy_setopt(_curl, CURLOPT_WRITEFUNCTION, guoke_request_callback);
    curl_easy_setopt(_curl, CURLOPT_WRITEDATA, &str_response);

    res = curl_easy_perform(_curl);

    for (int iter = 0; iter < 10; ++iter) {
        if (res == CURLE_OK) {
            break;
        } else {
            //Sleep(100);
            res = curl_easy_perform(_curl);
        }
    }

    std::vector<std::string> data_name;

    if (res == CURLE_OK) {

        Json::Reader reader;
        Json::Value v_all;
        Json::Value v_data;

        if (reader.parse(str_response, v_all)) {
            int v_code = v_all["code"].asInt();
            //printf("%d\n", v_code);

            if (v_code != 0) {
                printf("get data error!\n");
                return -1;
            }

            v_data = v_all["data"];

            for (int ii = 0; ii < v_data.size(); ii++) {
                TASK1 task1_data;
                std::stringstream str_name;
                std::string s_name;
                std::string _id = v_data[ii]["_id"].asString();
                task1_data.id = _id;

                int image_id = v_data[ii]["image_id"].asInt();
                task1_data.image_id = image_id;

                int image_idx = v_data[ii]["image_idx"].asInt();
                task1_data.image_idx = image_idx;

                int task_id = v_data[ii]["task_id"].asInt();
                task1_data.task_id = task_id;

                for (int jj = 0; jj < v_data[ii]["loc"]["coordinates"].size(); jj++) {
                    task1_data.xyzpoint.val[jj] = v_data[ii]["loc"]["coordinates"][jj].asDouble();
                }

                task1_return_data.push_back(task1_data);
            }
        }
    }

    return 0;

}


#endif
