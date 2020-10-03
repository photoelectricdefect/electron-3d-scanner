#ifndef SCANNERCALIB_H_
#define SCANNERCALIB_H_

#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace scanner {
    class scannercalib {
        public:            
            cv::Size board_size, square_size;
            Eigen::Hyperplane<double, 3> laser_plane;
            //int ncaps;

            scannercalib();
            
            void load();
    };
}

#endif
