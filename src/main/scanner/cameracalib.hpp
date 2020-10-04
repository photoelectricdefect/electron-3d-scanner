#ifndef CAMERACALIB_H_
#define CAMERACALIB_H_

#include <opencv2/highgui.hpp>
#include <json.hpp>

namespace scanner {
    struct jcameracalib {
        int ncaps;
        std::vector<double> K, D, square_size;
        std::vector<int> board_size;
    };

    void to_json(nlohmann::json& j, const jcameracalib& data);
    void from_json(const nlohmann::json& j, jcameracalib& data);
    
    class cameracalib {
        public:
            cv::Mat K, D;
            cv::Size board_size, square_size;
            int ncaps;
            
            cameracalib();

            void load();
            void save(std::string fpath);        
            // void load(std::string fpath);
    };
}

#endif
