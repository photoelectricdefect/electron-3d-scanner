#include <cameracalib.hpp>

namespace scanner {
    cameracalib::cameracalib() {};
    cameracalib::cameracalib(cv::Size board_size_, cv::Size square_size_, int ncaps_) : board_size(board_size_), square_size(square_size_), ncaps(ncaps_) {};

    //TODO: read configuration and saved data from files
    void cameracalib::load() {
        board_size = cv::Size(9, 6);
        square_size = cv::Size(23, 23);
        ncaps = 20;
    }
}
