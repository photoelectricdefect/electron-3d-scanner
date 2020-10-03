#include <scannercalib.hpp>

namespace scanner {
    scannercalib::scannercalib() {};

    //TODO: read configuration and saved data from files
    void scannercalib::load() {
        board_size = cv::Size(9, 6);
        square_size = cv::Size(23, 23);
        //ncaps = 20;
    }
}
