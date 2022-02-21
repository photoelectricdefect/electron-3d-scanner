#ifndef CAMERACALIBRATION_H_
#define CAMERACALIBRATION_H_

#include <opencv2/highgui.hpp>
#include <json.hpp>

namespace scanner {    
    class camera_calibration {
        public:
            // cv::Mat K, D;
            // cv::Size pattern_size=cv::Size(9,6);
            // double square_size=15.5;
            // int n_captures=35;

            cv::Mat K, D;
            cv::Size pattern_size;
            double square_size;
            int n_captures;

            camera_calibration();

            void create_calibration_file(std::string fpath);
            void save(std::string fpath);        
            void load(std::string fpath);
    };
}

#endif
