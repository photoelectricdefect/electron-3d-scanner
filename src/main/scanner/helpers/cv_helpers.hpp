#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <models/line_segment.hpp>
#include <string>
#include <vector>

namespace cv_helpers {

const uint8_t MIN_INTENSITY = 0;
const uint8_t MAX_INTENSITY = 255;
const cv::Vec3b MIN_INTENSITY_3C(0, 0, 0) ;
const cv::Vec3b MAX_INTENSITY_3C(255, 255, 255);

template<typename F>
F get_px1(const cv::Mat& img, int x, int y) {
	return img.ptr<F>(y)[x];
}

template<typename F>
void set_px1(cv::Mat& img, int x, int y, F intensity) {
	img.ptr<F>(y)[x] = intensity;
}

    template<typename type_in,typename type_out>
    void summed_area_table(const cv::Mat& in,cv::Mat& out) {
        for(size_t x=0;x<in.cols;x++) {
            type_out sum=get_px1<type_in>(in,x,0);

            if(x>0) sum+=get_px1<type_out>(out,x-1,0);  

            set_px1<type_out>(out,x,0,sum);
        }

        for(size_t y=0;y<in.rows;y++) {
            type_out sum=get_px1<type_in>(in,0,y);

            if(y>0) sum+=get_px1<type_out>(out,0,y-1);  

            set_px1<type_out>(out,0,y,sum);
        }

        for(size_t x=1;x<in.cols;x++) {
            for(size_t y=1;y<in.rows;y++) {
                type_out C =get_px1<type_in>(in,x,y),Ix=get_px1<type_out>(out,x-1,y),Iy=get_px1<type_out>(out,x,y-1),Ixy=get_px1<type_out>(out,x-1,y-1);
                type_out I=C+Ix+Iy-Ixy;
                set_px1<type_out>(out,x,y,I);            
            }
        }
    }

void set_px(cv::Mat& img, int x, int y, uint8_t intensity);
void set_px3C(cv::Mat& img, int x, int y, cv::Vec3b intensity);
uint8_t get_px(const cv::Mat& img, int x, int y);
cv::Vec3b get_px3C(const cv::Mat& img, int x, int y);
void sharpen(const cv::Mat& img, const cv::Mat& sharpened, float alpha, int threshold);
bool inside_polygon(const Eigen::Vector2d& a, const std::vector<line_segment>& sides);
void cut(cv::Mat& mask, cv::Size img_size, const std::vector<line_segment>& borders);
void crop(const cv::Mat& img,cv::Mat& cropped,const Eigen::Vector2d& q,const Eigen::Vector2d& s);
//static float masked_threshold(cv::Mat& img, cv::Mat mask, int type, int threshold);
size_t mat2buffer(cv::Mat& img, uint8_t*& data);
std::string mat2base64str(cv::Mat& img);
size_t mat2base64(cv::Mat& img, char*& data);
void PCA(cv::Mat& data, Eigen::MatrixXd& V, Eigen::MatrixXd& D, Eigen::Vector2d& O);
void ZhangSuen_thinning(const cv::Mat& binary, cv::Mat& out);
// void Bradley_thresholding(const cv::Mat& gray,cv::Mat& out,int wsize,int t,int thresh_min);

}