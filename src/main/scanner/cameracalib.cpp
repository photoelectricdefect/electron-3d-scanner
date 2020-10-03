#include <cameracalib.hpp>

namespace scanner {
    cameracalib::cameracalib() {};

    //TODO: read configuration and saved data from files
    void cameracalib::load() {
        board_size = cv::Size(9, 6);
        square_size = cv::Size(23, 23);
        ncaps = 20;
    }
}
