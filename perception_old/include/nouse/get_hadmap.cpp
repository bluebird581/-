#include <fstream>
#include "requst_1.h"
#include <time.h>
#include <curl.h>
//#include <types.h>
#include <easy.h>
#include <stdio.h>
//#ifdef __GNUC__
//#include <iconv.h>
//#endif
//#include "json/json.h"

GuokeRequest::GuokeRequest() {
    _curl = curl_easy_init();

    if (!_curl) {
        std::cout << "[error] could not initialize the guoke communication model" << std::endl;
    }
}

GuokeRequest::~GuokeRequest() {
    curl_easy_cleanup(_curl);
}

int GuokeRequest::split(const std::string& str,
                        std::vector<std::string>& ret, const std::string sep) {
    if (str.empty()) {
        return 0;
    }

    std::string tmp;
    std::string::size_type pos_begin = str.find_first_not_of(sep);
    std::string::size_type comma_pos = 0;

    while (pos_begin != std::string::npos) {
        comma_pos = str.find(sep, pos_begin);

        if (comma_pos != std::string::npos) {
            tmp = str.substr(pos_begin, comma_pos - pos_begin);
            pos_begin = comma_pos + sep.length();
        } else {
            tmp = str.substr(pos_begin);
            pos_begin = comma_pos;
        }

        if (!tmp.empty()) {
            ret.push_back(tmp);
            tmp.clear();
        }
    }

    return 0;
}
static size_t guoke_request_callback(void* ptr, size_t size, size_t nmemb, std::string* stream) {
    stream->append((char*)ptr, size * nmemb);
    stream->append("\0");

    return size * nmemb;
}
//任务1
/*
int GuokeRequest::do_request_task1(std::string url_name, std::string pic_id,std::vector<stcv::TASK1>& task1_return_data)
{
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
    curl_slist *ptr_curl_slist = curl_slist_append(NULL,
        "Content-Type:application/json;charset=UTF-8");
    curl_easy_setopt(_curl, CURLOPT_HTTPHEADER, ptr_curl_slist);
    //curl_easy_setopt(_curl, CURLOPT_POSTFIELDS, json_data);
    curl_easy_setopt(_curl, CURLOPT_WRITEFUNCTION, guoke_request_callback);
    curl_easy_setopt(_curl, CURLOPT_WRITEDATA, &str_response);

    res = curl_easy_perform(_curl);

    for (int iter = 0; iter < 10; ++iter)
    {
        if (res == CURLE_OK)
        {
            break;
        }
        else
        {
            //Sleep(100);
            res = curl_easy_perform(_curl);
        }
    }
    std::vector<std::string> data_name;
    if (res == CURLE_OK)
    {

        Json::Reader reader;
        Json::Value v_all;
        Json::Value v_data;

        if (reader.parse(str_response, v_all))
        {
            int v_code = v_all["code"].asInt();
            printf("%d\n", v_code);

            if (v_code != 0)
            {
                printf("get data error!\n");
                return 0;
            }

            v_data = v_all["data"];
            for (int ii = 0; ii < v_data.size(); ii++)
            {
                stcv::TASK1 task1_data;
                std::stringstream str_name;
                std::string s_name;
                std::string _id = v_data[ii]["_id"].asString();
                task1_data._id = _id;

                int image_id = v_data[ii]["image_id"].asInt();
                task1_data.image_id = image_id;

                int image_idx = v_data[ii]["image_idx"].asInt();
                task1_data.image_idx = image_idx;

                int task_id = v_data[ii]["task_id"].asInt();
                task1_data.task_id = task_id;
                std::vector<double> ps_v;
                for (int jj = 0; jj < v_data[ii]["loc"]["coordinates"].size(); jj++)
                {
                    double v_d = v_data[ii]["loc"]["coordinates"][jj].asDouble();
                    ps_v.push_back(v_d);
                }
                task1_data.xypoint = ps_v;
                task1_return_data.push_back(task1_data);
            }
        }
    }

    return 0;

}
*/

//任务2
/*
int GuokeRequest::do_request_task2(std::string url_name, std::string pic_id,stcv::TASK2& task2_return_data)
{
    std::vector<std::vector<cv::Point2d>> return_data;
    int num = 20;
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
    curl_slist *ptr_curl_slist = curl_slist_append(NULL,
        "Content-Type:application/json;charset=UTF-8");
    curl_easy_setopt(_curl, CURLOPT_HTTPHEADER, ptr_curl_slist);
    //curl_easy_setopt(_curl, CURLOPT_POSTFIELDS, json_data);
    curl_easy_setopt(_curl, CURLOPT_WRITEFUNCTION, guoke_request_callback);
    curl_easy_setopt(_curl, CURLOPT_WRITEDATA, &str_response);

    res = curl_easy_perform(_curl);

    for (int iter = 0; iter < 10; ++iter)
    {
        if (res == CURLE_OK)
        {
            break;
        }
        else
        {
            //Sleep(100);
            res = curl_easy_perform(_curl);
        }
    }
    std::vector<std::vector<cv::Point2d>>  data_name(num);
    std::vector<std::vector<std::string>>  id_name(num);
    if (res == CURLE_OK)
    {

        Json::Reader reader;
        Json::Value v_all;
        Json::Value v_data;

        if (reader.parse(str_response, v_all))
        {
            int v_code = v_all["code"].asInt();
            printf("%d\n", v_code);

            if (v_code != 0)
            {
                printf("get data error!\n");
                return 0;
            }

            v_data = v_all["data"]["lane_line"];
            std::vector<double> pv;
            int idx_base = 1;
            for (int ii = 0; ii < v_data.size(); ii++)
            {
                int idx = abs(v_data[ii]["lane_idx"].asInt());
                std::string idname = v_data[ii]["_id"].asString();
                id_name[idx-1].push_back(idname);
                printf("%s\n",idname.c_str());
                std::cout<< v_data[ii]["line_segment"] <<std::endl;
                for (int jj = 0; jj < v_data[ii]["line_segment"].size(); jj++) //[] []
                {

                    std::cout<< v_data[ii]["line_segment"][jj] <<std::endl;
                    pv.clear();
                    for(int nn = 0; nn < 2; nn++) // [] //v_data[ii]["line_segment"][jj].size()
                    {
                        //第三位是高度信息
                        double v = v_data[ii]["line_segment"][jj][nn].asDouble();
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
                            data_name[idx - 1].push_back(pvt);
                        }
                    }
                }
            }

        }
    }
    std::vector<std::vector<std::string>> result_ids;
    for (int i = 0; i < num; i++)
    {
        if (data_name[i].size())
        {
            return_data.push_back(data_name[i]);
            result_ids.push_back(id_name[i]);
        }
    }

    task2_return_data.points = return_data;
    task2_return_data._id = result_ids;
    return 0;

}
*/
