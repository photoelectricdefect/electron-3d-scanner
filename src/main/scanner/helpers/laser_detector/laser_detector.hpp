#ifndef LASERDET_H_
#define LASERDET_H_

#include <vector>
#include <limits>
#include <thread>
#include "opencv2/core.hpp"

/**
 * @brief  Class for detection of laser line in image, using laser/no-laser image pairs
 * @note   
 * @retval None
 */
class laser_detector
{
private:
    cv::Mat im_;
    std::vector<std::vector<long int>> left_csum_;
    std::vector<std::vector<uint8_t>> imvec_;
    std::vector<std::vector<int>> peaks_;
    int nthreads_available_,nthreads_max_; 
public:
    //TODO: Update so that a laser/no-laser image pair is used to generate the required difference image for detection
    /**
     * @brief  Constructor
     * @note   
     * @param  im: Laser image 
     * @param  model_path: Path svm to model
     * @param  nthreads_max: Maximum number of threads used in detection
     * @retval 
     */
    laser_detector(const cv::Mat& im,const int nthreads_max);
    laser_detector(const cv::Mat& im_laser,const cv::Mat& im_nolaser,const int channel,const int nthreads_max);
    ~laser_detector();

    /**
     * @brief  Detects laser line, you can also play around with the parameters used in the algorithm, they are defined in laser_detection.cpp 
     * @note   
     * @param  vstep: y-axis resolution, ex.: a step of 2 will detect laser line in every second row
     * @param  threshold: Minimum intensity of laser line peak
     * @param  peak_sharpness: how sharp a peak should be, should be in range [0,PI], lower is sharper   
     * @param  min_winw: Minimum peak search window size
     * @param  max_winw: Maximum peak search window size
     * @retval 
     */
    std::vector<std::vector<int>> detect(const size_t vstep,const int threshold=20,const double peak_sharpness=M_PI/2.5,const int min_winw=15,const int max_winw=20);
};

#endif