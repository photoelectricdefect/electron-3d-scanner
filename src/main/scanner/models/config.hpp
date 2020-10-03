#ifndef CONFIG_H_
#define CONFIG_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <json.hpp>

namespace scanner {    
    struct calib_data {
        std::vector<double> K, D, square_size;
        std::vector<int> board_size;
    };

    void to_json(nlohmann::json& j, const calib_data& data);
    void from_json(const nlohmann::json& j, calib_data& data);

    class config {
        public:
            void save_calib(const cv::Mat& K, const cv::Mat& D);        
            void load_calib(cv::Mat& K, cv::Mat& D);        
    };
}

#endif