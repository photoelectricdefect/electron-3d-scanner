#ifndef SCANNERCALIB_H_
#define SCANNERCALIB_H_

#include <opencv2/core.hpp>
#include <json.hpp>
#include <Eigen/Dense>

namespace scanner {
    struct jscannercalib {
        std::vector<double> laser_plane;
    };

    void to_json(nlohmann::json& j, const jscannercalib& data);
    void from_json(const nlohmann::json& j, jscannercalib& data);

    class scannercalib {
        public:            
            cv::Size board_size=cv::Size(9,6),square_size=cv::Size(23,23);
            Eigen::Hyperplane<double,3> laser_plane;
            int captures=20;

            scannercalib();
            
            bool load(std::string fpath);
            void save(std::string fpath);        
    };
}

#endif
