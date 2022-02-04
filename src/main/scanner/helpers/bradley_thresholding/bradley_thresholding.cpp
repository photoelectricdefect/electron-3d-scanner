#include "bradley_thresholding.hpp"
#include "summed-area-table/summed_area_table.hpp"
#include "iostream"

bradley_thresholding::bradley_thresholding(/* args */)
{
}

void bradley_thresholding::threshold(const cv::Mat& im,cv::Mat& out,int wsize,int t,int thresh_min) {
    std::vector<std::vector<uint8_t>> imvec(im.rows);
    
    for (size_t i = 0; i < im.rows; i++)
    {
        imvec[i]=std::vector<uint8_t>(im.ptr<uint8_t>(i),im.ptr<uint8_t>(i)+im.cols);        
    }
    
    summed_area_table<uint8_t,long> sat(imvec);
    int r=wsize/2;

    for(size_t x=0;x<im.cols;x++) {
        for(size_t y=0;y<im.rows;y++) {
            int predicted=im.ptr<uint8_t>(y)[x];
            double mean=sat.mean(x,y,r);

            if(predicted<=mean*(100-t)/100||predicted<=thresh_min) out.ptr<uint8_t>(y)[x]=0;
            else out.ptr<uint8_t>(y)[x]=255;
        }
    }
}
