#ifndef STCV_GUOKE_REQUEST_H
#define STCV_GUOKE_REQUEST_H

#include <opencv2/opencv.hpp>
#include <set>
//#ifdef __GNUC__
//#include "json/json.h"
//#else
//#include "json.h"
//#endif

#include "basic_struct.h"

//typedef struct TASK1
//{
//    std::string _id;
//    int image_id;
//    int image_idx;
//    int task_id;
//    cv::Vec3d xyzpoint;
//
//};

class HadmapData {

public:

    //GuokeRequest();
    //~GuokeRequest();

    cv::Point2d lonlat_to_mercator(const cv::Point2d& lonlat) {
        cv::Point2d mercator;

        mercator.x = lonlat.x * 20037508.34 / 180;
        mercator.y = log(tan((90 + lonlat.y) * CV_PI / 360)) / (CV_PI / 180);
        mercator.y = mercator.y * 20037508.34 / 180;

        return mercator;
    }

    cv::Point2d mercator_to_lonlat(const cv::Point2d& mercator) {
        cv::Point2d lonlat;

        lonlat.x = mercator.x / 20037508.34 * 180;
        lonlat.y = mercator.y / 20037508.34 * 180;
        lonlat.y = 180 / CV_PI * (2 * atan(exp(lonlat.y * CV_PI / 180)) - CV_PI / 2);

        return lonlat;
    }

    //int do_request_task1(std::string url_name, std::string pic_id,std::vector<TASK1>& task1_return_data);
    int do_request_task2(std::string url_name, std::vector<LANE_COMBO>& task2_return_data);
    int do_request_task2_arrows(std::string url_name, std::vector<ARROW>& task2_return_data);
private:

    Timer _timer;

    //CURL* _curl;

};
//
//GuokeRequest::GuokeRequest()
//{
//    _curl = curl_easy_init();
//
//    if (!_curl)
//    {
//        std::cout << "[error] could not initialize the guoke communication model" << std::endl;
//    }
//}
//
//GuokeRequest::~GuokeRequest()
//{
//    curl_easy_cleanup(_curl);
//}
//
//
//static size_t guoke_request_callback(void *ptr, size_t size, size_t nmemb, std::string *stream)
//{
//    stream->append((char*)ptr, size * nmemb);
//    stream->append("\0");
//
//    return size * nmemb;
//}
////任务1
//int HadmapData::do_request_task1(std::string url_name, std::string pic_id, std::vector<TASK1>& task1_return_data)
//{
//    //std::cout << url_name << std::endl;
//    CURLcode res;
//    char json_data[1024];
//    memset(json_data, 0, sizeof(json_data));
//    std::string json_str = "{";
//    json_str += "\"pic_id\" : \"";
//    json_str += pic_id;
//    json_str += "\"";
//    json_str += "}";
//    std::string str_response;
//
//#ifdef __GNUC__
//    strcpy(json_data, json_str.c_str());
//#else
//    strcpy_s(json_data, json_str.c_str());
//#endif
//
//    curl_easy_setopt(_curl, CURLOPT_TIMEOUT, 1);
//    curl_easy_setopt(_curl, CURLOPT_URL, url_name.c_str());
//    curl_slist *ptr_curl_slist = curl_slist_append(NULL,
//                                 "Content-Type:application/json;charset=UTF-8");
//    curl_easy_setopt(_curl, CURLOPT_HTTPHEADER, ptr_curl_slist);
//    //curl_easy_setopt(_curl, CURLOPT_POSTFIELDS, json_data);
//    curl_easy_setopt(_curl, CURLOPT_WRITEFUNCTION, guoke_request_callback);
//    curl_easy_setopt(_curl, CURLOPT_WRITEDATA, &str_response);
//
//    res = curl_easy_perform(_curl);
//
//    for (int iter = 0; iter < 10; ++iter)
//    {
//        if (res == CURLE_OK)
//        {
//            break;
//        }
//        else
//        {
//            //Sleep(100);
//            res = curl_easy_perform(_curl);
//        }
//    }
//    std::vector<std::string> data_name;
//    if (res == CURLE_OK)
//    {
//
//        Json::Reader reader;
//        Json::Value v_all;
//        Json::Value v_data;
//
//        if (reader.parse(str_response, v_all))
//        {
//            int v_code = v_all["code"].asInt();
//            //printf("%d\n", v_code);
//
//            if (v_code != 0)
//            {
//                printf("get data error!\n");
//                return 0;
//            }
//
//            v_data = v_all["data"];
//            for (int ii = 0; ii < v_data.size(); ii++)
//            {
//                TASK1 task1_data;
//                std::stringstream str_name;
//                std::string s_name;
//                std::string _id = v_data[ii]["_id"].asString();
//                task1_data._id = _id;
//
//                int image_id = v_data[ii]["image_id"].asInt();
//                task1_data.image_id = image_id;
//
//                int image_idx = v_data[ii]["image_idx"].asInt();
//                task1_data.image_idx = image_idx;
//
//                int task_id = v_data[ii]["task_id"].asInt();
//                task1_data.task_id = task_id;
//                for (int jj = 0; jj < v_data[ii]["loc"]["coordinates"].size(); jj++)
//                {
//                    task1_data.xyzpoint.val[jj] = v_data[ii]["loc"]["coordinates"][jj].asDouble();
//                }
//                task1_return_data.push_back(task1_data);
//            }
//        }
//    }
//
//    return 0;
//
//}


//任务2
/*
int HadmapData::do_request_task2(std::string url_name,
                                   std::vector<LANE_COMBO> &task2_return_data)
{
    //std::cout << url_name << std::endl;
    //FILE *fp = fopen("sb.txt", "w");
    //fprintf(fp, "%s", url_name.c_str());
    //fclose(fp);

    _timer.do_start_timer();

    std::map<int, int> count_idx_map;
    count_idx_map.clear();


    Json::Reader reader;
    Json::Value v_all;
    //Json::Value v_data;

    if (reader.parse(url_name, v_all))
    {
        _timer.do_end_timer("    [-] misc time: ");

        int v_code = v_all["code"].asInt();
        //printf("%d\n", v_code);

        if (v_code != 0)
        {
            return -1;
        }

        // 1. scan each line
        Json::Value &v_data = v_all["data"]["lane_line"];

        for (int ii = 0; ii < v_data.size(); ii++)
        {
            int lane_count = v_data[ii]["lane_count"].asInt() + 1;
            int idx = abs(v_data[ii]["lane_idx"].asInt());
            int lane_leftline_type =  v_data[ii]["lane_leftline_type"].asInt();

            //std::cout << idx << "/" << lane_count << std::endl;

            // 1.2. if find, push back, or new
            int sz = task2_return_data.size();
            std::map<int, int>::iterator iter = count_idx_map.find(lane_count);

            int idx_combo = -1;
            if (iter == count_idx_map.end())
            {
                count_idx_map.insert(std::make_pair(lane_count, sz));

                LANE_COMBO lane_combo;
                lane_combo.lanes.resize(lane_count);
                task2_return_data.push_back(lane_combo);

                idx_combo = task2_return_data.size() - 1;
            }
            else
            {
                idx_combo = iter->second;
            }

            task2_return_data[idx_combo].lanes[idx - 1].status = 0;
            task2_return_data[idx_combo].lanes[idx - 1].type = lane_leftline_type != 3;
            // 1.1. scan pts of each line

            for (int jj = 0; jj < v_data[ii]["line_segment"].size(); ++jj)
            {
                //std::cout << v_data[ii]["line_segment"][jj].size() << std::endl;
                WEIGHTEDPOINT pt;
                int nn = 0;
                pt.pt_d.x = v_data[ii]["line_segment"][jj][nn].asDouble();
                pt.pt_d.y = v_data[ii]["line_segment"][jj][1].asDouble();
                pt.pt_d = lonlat_to_mercator(pt.pt_d);
                task2_return_data[idx_combo].lanes[idx - 1].pts_ipm.push_back(pt);
            }


        }

        _timer.do_end_timer("    [-] misc time: ");

        for (int i = 0; i < task2_return_data.size(); ++i)
        {
            for (int j = 0; j < task2_return_data[i].lanes.size(); ++j)
            {
                if (task2_return_data[i].lanes[j].pts_ipm.size() < 2)
                {
                    task2_return_data[i].lanes.erase(task2_return_data[i].lanes.begin() + j);
                    --j;
                }
            }
        }

    }


    return 0;

}

int HadmapData::do_request_task2_arrows(std::string url_name, std::vector<ARROW> &task2_return_data)
{

    int num = 20;

    std::vector<std::vector<cv::Point2d> >  data_name(num);
    std::vector<std::vector<std::string> >  id_name(num);


    Json::Reader reader;
    Json::Value v_all;
    //Json::Value v_data;

    if (reader.parse(url_name, v_all))
    {
        int v_code = v_all["code"].asInt();
        //printf("%d\n", v_code);

        if (v_code != 0)
        {
            return -1;
        }

        Json::Value &v_data = v_all["data"]["roadmark_arrow"];
        std::vector<double> pv;
        int idx_base = 1;

        for (int ii = 0; ii < v_data.size(); ii++)
        {
            ARROW arrow_t;
            int idx = abs(v_data[ii]["lane_idx"].asInt());

            std::string idname = v_data[ii]["_id"].asString();
            arrow_t.id = idname;

            //std::cout<< v_data[ii]["boundary"] <<std::endl;
            //if (v_data[ii]["boundary"].size() != 14)
            //{
            //    continue;
            //}
            for (int jj = 0; jj < v_data[ii]["boundary"].size(); jj++) //[] []
            {

                //std::cout<< v_data[ii]["boundary"][jj] <<std::endl;
                pv.clear();
                for(int nn = 0; nn < 2; nn++) // [] //v_data[ii]["line_segment"][jj].size()
                {
                    double v = v_data[ii]["boundary"][jj][nn].asDouble();
                    pv.push_back(v);
                }
                if (pv.size() < 2)
                {
                    printf("lane data error\n");
                }
                else
                {
                    for (int nn = 0; nn < pv.size()/2; nn++)
                    {
                        cv::Point2d pvt = cv::Point2d(pv[nn], pv[nn+1]);
                        arrow_t.pts_ipm.push_back(lonlat_to_mercator(pvt));
                    }
                }
            }
            task2_return_data.push_back(arrow_t);
        }

    }


    return 0;

}
*/

#endif
