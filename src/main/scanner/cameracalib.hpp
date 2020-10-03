#ifndef CAMERACALIB_H_
#define CAMERACALIB_H_

#include <opencv2/highgui.hpp>

namespace scanner {
    class cameracalib {
        public:
            cv::Mat K, D;
            cv::Size board_size, square_size;
            int ncaps;
            
            cameracalib();
            cameracalib(cv::Size board_size_, cv::Size square_size_, int ncaps_);
            
            void load();
    };
}

#endif
