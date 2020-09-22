#include <cameracalib.hpp>

namespace scanner {
    cameracalib::cameracalib() {};
    cameracalib::cameracalib(const cv::Size& _board_size, const cv::Size& _square_size, int _ncaps) : board_size(_board_size), square_size(_square_size), ncaps(_ncaps) {};

    //TODO: read configuration and saved data from files
    void cameracalib::load() {
        board_size = cv::Size(9, 6);
        square_size = cv::Size(23, 23);
        ncaps = 20;
    }
}
