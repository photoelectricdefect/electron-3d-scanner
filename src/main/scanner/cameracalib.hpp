#ifndef CAMERACALIB_H_
#define CAMERACALIB_H_

#include <opencv2/highgui.hpp>
#include <json.hpp>

namespace scanner {
    struct jcameracalib {
        std::vector<double> K,D;
    };

    void to_json(nlohmann::json& j, const jcameracalib& data);
    void from_json(const nlohmann::json& j, jcameracalib& data);
    
    class cameracalib {
        public:
            cv::Mat K, D;
            cv::Size pattern_size=cv::Size(9,6);
            const double square_width=16;
            const int captures=35;
            
            cameracalib();

            void save(std::string fpath);        
            bool load(std::string fpath);
    };
}

#endif
