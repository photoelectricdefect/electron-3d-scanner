#include <scannercalib.hpp>
#include <opencv2/ximgproc/slic.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <models/line_segment.hpp>

namespace scanner {
    scannercalib::scannercalib() {};
    scannercalib::scannercalib(cv::Size board_size_, cv::Size square_size_/*, int ncaps_*/) : board_size(board_size_), square_size(square_size_)/*, ncaps(ncaps_)*/ {}
}
