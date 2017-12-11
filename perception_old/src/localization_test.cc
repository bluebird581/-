#include <string>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "camera_localization.h"

cv::Point2d lonlat_to_mercator_zhu(const cv::Point2d &lonlat)
{
        cv::Point2d mercator;

        mercator.x = lonlat.x * 20037508.34 / 180;
        mercator.y = log(tan((90 + lonlat.y) * CV_PI / 360)) / (CV_PI / 180);
        mercator.y = mercator.y * 20037508.34 / 180;

        return mercator;
}
// ../config/changcheng_0811.config ../data/testdata_0824/test_list_jga.txt ../data/testdata_0824/image/
int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cout << "Input Error!" << std::endl;
        std::cout << "Usage: ./localization_test ../config/apollo_release.config \
        ../data/testdata1/test_list.txt ../data/testdata1/image/" << std::endl;
        std::cout << "   or: ./localization_test ../config/apollo_release.config \
        ../data/testdata2/test_list.txt ../data/testdata2/image/" << std::endl;
        return -1;
    }

    std::string config_file = argv[1];
    std::string test_file_name = argv[2];
    std::string img_dir = argv[3];
    
    apollo::localization::CameraLocalization loc;

	if (!loc.init(config_file)) {
        std::cout << "initializing failed" << std::endl;
        return -1;
    }
    
    std::ifstream test_file(test_file_name.c_str());
    std::string imname;
    double l_x = 0;
    double l_y = 0;
    double h_x = 0;
    double h_y = 0;
    double heading = 0;
    std::ofstream fout;
    fout.open("result.txt");
    std::ofstream rout;
    rout.open("ratio_dis.txt", std::ios::out);
    //  while (test_file >> imname >> l_x >> l_y >> h_x >> h_y >> heading) {
    
    for (int ite = 0; ite < 16; ++ite) { //6740 //6085
        for (int i = 0; i < 1; ++i){
            test_file >> imname >> l_x >> l_y >> h_x >> h_y >> heading;
        }
    //     if (ite < 660 || ite > 675)
    //         continue;
        std::cout << imname << std::endl;
        std::string impath = img_dir + imname;
        cv::Mat image = cv::imread(impath);
        if (image.empty()) {
        std::cout << "[ERROR] Can not open image " << impath << std::endl;
        continue;
        }
        apollo::localization::PosInfo input_pos;
        apollo::localization::PosInfo output_pos;
        input_pos.longitude = l_x; 
        input_pos.latitude = l_y;  
        if (!loc.get_ego_position(image, input_pos, output_pos)) {
        //xhzhu 0717 ,it's not proper to do nothing while fail to localization, output raw gps is a choice. // effect too much, need other way.
        std::cout << "hshshshshh" << std::endl;
        continue;
        //output_pos.longitude = l_x;
        //output_pos.latitude = l_y;
        }
        std::cout << "\n\n\n";
        std::cout << "------------lines_det:  " << output_pos.lanes_det.size() << std::endl;
        std::cout << "------------arrows_det: " << output_pos.arrows_det.size() << "\n\n\n";

        std::cout << "------------lines_had:  " << output_pos.lanes_had.lanes.size() << std::endl;
        std::cout << "------------arrows_had: " << output_pos.arrows_had.size() << "\n\n\n";
        //std::cout << std::setiosflags(std::ios::fixed);
        //std::cout << "[INFO] GROUNDTRUTH (" << std::setprecision(10)
        //    << h_x << ", " << std::setprecision(10) << h_y << ")\t"
        //    << " LOCALIZATION (" << output_pos.longitude << ", "
        //    << output_pos.latitude << ")" << std::endl;
        //std::cout << "[INFX] CHANNEL: " << output_pos.channel << "\tABs Angle: " << 
        //    output_pos.heading_absolute << "\tRelative Angle: " 
        //    << output_pos.heading_relative << std::endl;
        //std::cout << std::endl;
        fout << imname << "  "  << std::setprecision(10)
            << h_x << "  " << std::setprecision(10) << h_y << "  "
            << output_pos.longitude << "  "
            << output_pos.latitude  << "  "
            << heading << "  "
            << output_pos.heading_absolute << "  "
            << output_pos.heading_relative << "  "
            << output_pos.heading_absolute + output_pos.heading_relative - heading << "  "
            << output_pos.channel
            << std::endl;
        //if (output_pos.camera_dis_2_side.x != -1) {
        //std::cout << "111 "<< imname << std::endl;
        rout << imname << "  " << output_pos.camera_dis_2_side.x << "  " 
            << output_pos.camera_dis_2_side.y << "  " << output_pos.hadmap_dis_2_side.x 
            << "  " << output_pos.hadmap_dis_2_side.y << "  ";
        //}
        //xhzhu 0901 calc error and output
        cv::Point2d low_pt = cv::Point2d(l_x, l_y);
        cv::Point2d hig_pt = cv::Point2d(h_x, h_y);
        low_pt = lonlat_to_mercator_zhu(low_pt);
        hig_pt = lonlat_to_mercator_zhu(hig_pt);
        //std::cout << low_pt.x << "  " << low_pt.y << "  " << hig_pt.x << "  " << hig_pt.y << std::endl;
        //std::cout << output_pos.rotate_angle << std::endl;
        //hig_pt.x = (hig_pt.x - low_pt.x) * 16;
        //hig_pt.y = (hig_pt.y - low_pt.y) * 16;
        hig_pt.x = cos(output_pos.rotate_angle) * hig_pt.x 
                    - sin(output_pos.rotate_angle) * hig_pt.y;
        hig_pt.y = -(sin(output_pos.rotate_angle) * hig_pt.x 
                    + cos(output_pos.rotate_angle) * hig_pt.y);
        low_pt.x = cos(output_pos.rotate_angle) * low_pt.x 
                    - sin(output_pos.rotate_angle) * low_pt.y;
        low_pt.y = -(sin(output_pos.rotate_angle) * low_pt.x 
                    + cos(output_pos.rotate_angle) * low_pt.y);
        hig_pt.x = (hig_pt.x - low_pt.x) * 16 + 224; //23.9
        hig_pt.y = (hig_pt.y - low_pt.y) * 16;
        rout << output_pos.match_x << "  " << output_pos.short_x 
            << "  " << output_pos.long_x << " " << output_pos.final_x 
            << "  " << hig_pt.x << std::endl;
//#ifdef localization_display
//        cv::waitKey(0);
//#endif
    }
    fout.close();
    rout.close();
    return 0;
}
