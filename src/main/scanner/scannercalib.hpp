#ifndef SCANNERCALIB_H_
#define SCANNERCALIB_H_

#include <opencv2/core.hpp>
#include <json.hpp>
#include <Eigen/Dense>

namespace scanner {
    struct jscannercalib {
        // int ncaps;
        std::vector<double> square_size;
        std::vector<int> board_size;
    };

    void to_json(nlohmann::json& j, const jscannercalib& data);
    void from_json(const nlohmann::json& j, jscannercalib& data);

    class scannercalib {
        public:            
            cv::Size board_size, square_size;
            Eigen::Hyperplane<double, 3> laser_plane;
            //int ncaps;

            scannercalib();
            
            void load();
            void save(std::string fpath);        
            // void load(std::string fpath);
    };
}

#endif
