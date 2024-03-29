#ifndef CAMERACALIBRATION_H_
#define CAMERACALIBRATION_H_

#include <opencv2/highgui.hpp>
#include <boost/thread.hpp>
#include <json.hpp>

namespace scanner {    
    class camera_calibration {
        public:
            cv::Mat K, D;
            cv::Size pattern_size;
            double square_size;
            int n_captures;

            camera_calibration();

            void create_calibration_file(const std::string& fpath);
            void save(const std::string& fpath);        
            void load(const std::string& fpath);
    };
}

#endif
