#ifndef SCANNERCALIB_H_
#define SCANNERCALIB_H_

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>

namespace scanner {
    class scannercalib {
        public:            
            cv::Size board_size, square_size;
            //int ncaps;

            scannercalib();
            scannercalib(cv::Size board_size_, cv::Size square_size_/*, int ncaps_*/);            
    };
}

#endif
