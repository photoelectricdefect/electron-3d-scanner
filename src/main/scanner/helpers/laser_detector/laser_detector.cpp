#include "laser_detector.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "dynp_partition.hpp"
#include <opencv2/objdetect.hpp>
#include <Eigen/Dense>

const double peak_lwr_bnd_coeff=0.75;
const int blur_winr=7;

laser_detector::laser_detector(const cv::Mat& im_laser,const cv::Mat& im_nolaser,const int channel,const int nthreads_max):nthreads_max_(nthreads_max) {
    cv::Mat base_colors[3],diff;
    cv::absdiff(im_laser,im_nolaser,diff);
    cv::split(diff,base_colors);
    nthreads_available_=std::thread::hardware_concurrency();
    left_csum_=std::vector<std::vector<long int>>(base_colors[channel].rows,std::vector<long int>(base_colors[channel].cols));
    imvec_=std::vector<std::vector<uint8_t>>(base_colors[channel].rows,std::vector<uint8_t>(base_colors[channel].cols));
    cv::medianBlur(base_colors[channel],im_,blur_winr);

    for (size_t i = 0; i < im_.rows; i++)
    {
        for (size_t j = 0; j < im_.cols; j++)
        {
            uint8_t C=im_.ptr<uint8_t>(i)[j];
            imvec_[i][j]=C;
            left_csum_[i][j]=j==0?(long int)C:(long int)left_csum_[i][j-1]+(long int)C;
        }   
    }
}

laser_detector::laser_detector(const cv::Mat& im,const int nthreads_max):nthreads_max_(nthreads_max)
{
    nthreads_available_=std::thread::hardware_concurrency();
    left_csum_=std::vector<std::vector<long int>>(im.rows,std::vector<long int>(im.cols));
    imvec_=std::vector<std::vector<uint8_t>>(im.rows,std::vector<uint8_t>(im.cols));
    cv::medianBlur(im,im_,blur_winr);

    for (size_t i = 0; i < im_.rows; i++)
    {
        for (size_t j = 0; j < im_.cols; j++)
        {
            uint8_t C=im_.ptr<uint8_t>(i)[j];
            imvec_[i][j]=C;
            left_csum_[i][j]=j==0?(long int)C:(long int)left_csum_[i][j-1]+(long int)C;
        }   
    }
}

laser_detector::~laser_detector()
{

}

template<typename T>
T get_sum(const std::vector<T>& csum,const int& from,const int& to) {
    return from>0?csum[to]-csum[from-1]:csum[to];
};

template<typename T>
double get_mean(const std::vector<T>& csum,const int& from,const int& to,const int& n) {
    return static_cast<double>(get_sum(csum,from,to))/n;
};

typedef struct partitioning_parameters
{
    std::vector<long int> csum;
} partitioning_parameters;

double peak_fitness(std::vector<uint8_t> data,int from, int to,void*ptr=nullptr) {
    int n=to-from+1;
    double cost=0;
    partitioning_parameters* pms=(partitioning_parameters*)ptr;
    int y_to=data[to],y_from=data[from];
    int baseline=std::min(y_to,y_from);
    int winr=n/2;
    int symm_sum=0,symm_diff=0;

    for (size_t i=0;i<winr;i++)
    {
        int ly=data[from+i],ry=data[to-i];
        int lydiff=ly>=baseline?ly-baseline:0,
        rydiff=ry>=baseline?ry-baseline:0;
        symm_sum+=lydiff+rydiff;
        symm_diff+=abs(lydiff-rydiff);
    }

    if(symm_sum==0) return 0;
        
    double symm_coeff=static_cast<double>(symm_diff)/symm_sum;   
    symm_coeff=std::max(symm_coeff,1e-9);
    double mean=get_mean(pms->csum,from,to,n);
    cost=-log(symm_coeff)*n*pow(mean,2);
    
    return cost;
}

std::vector<std::vector<int>> laser_detector::detect(const size_t vstep,const int threshold,const double peak_sharpness,const int min_winw,const int max_winw)
{
    std::vector<std::vector<int>> laser_pts;
    size_t rows=im_.rows,cols=im_.cols;
    int nthreads=nthreads_available_>nthreads_max_?nthreads_max_:nthreads_available_;
    std::vector<std::thread> tpool;
    std::mutex mtx;
    cv::Mat hist=cv::Mat::zeros(cv::Size(1600,1200),CV_8UC1);
    int tspacing=nthreads*vstep;
    const int winr=max_winw/2;

    for (size_t o = 0; o < nthreads; o++)
    {
        const auto fn=[this,offset=o,nthreads,vstep,tspacing,winr,threshold,peak_sharpness,min_winw,max_winw,&laser_pts,&mtx,&hist](){
            for(size_t h=0,i=h*tspacing+offset*vstep; i < im_.rows; h++,i=h*tspacing+offset*vstep)
            {
                partitioning_parameters* pms=new partitioning_parameters;
                pms->csum=left_csum_[i];
                dynp_partition<uint8_t> dp;
                std::vector<int> partitions=dp.partition(imvec_[i],peak_fitness,min_winw,max_winw,pms);
                delete pms;
                
                for (size_t j=0;j<partitions.size();j++)
                {
                    int n=j<partitions.size()-1?partitions[j+1]-partitions[j]:imvec_[i].size()-partitions[j];                 
                    int from=partitions[j],to=partitions[j]+n-1;
                    double cog_x0=0,cog_I=0;
                    int peak_I=INT_MIN;
                    std::vector<int> ns(UINT8_MAX+1,0);
                    std::vector<int> xs(UINT8_MAX+1,0);

                    for(int e = from; e <= to; e++)
                    {
                        ns[imvec_[i][e]]++;
                        xs[imvec_[i][e]]+=e;
                        peak_I=std::max(peak_I,(int)imvec_[i][e]);
                    }
    
                    int peak_lwr_bnd=peak_lwr_bnd_coeff*peak_I;
                    int n0=0;

                    for(int e=peak_lwr_bnd;e<=peak_I;e++)
                    {
                        cog_x0+=xs[e];
                        cog_I+=e*ns[e];
                        n0+=ns[e];
                    }

                    cog_x0/=n0;
                    cog_I/=n0;

                    if(cog_I<threshold) 
                        continue;

                    double center=(from+to)*0.5;
                    double cog_x1=xs[peak_I]/ns[peak_I];                   
                    Eigen::MatrixXd L(winr,1),bl(winr,1),
                    R(winr,1),br(winr,1);
                    int A=center-winr,B=center+winr;
                    A=A>=0?A:0,B=B<imvec_[i].size()?B:imvec_[i].size()-1;
                    int AB=B-A,C=(A+B)/2;

                    for (int e = A; e <= C-1; e++)
                    {
                        L(e-A,0)=e-cog_x1;
                        bl(e-A,0)=((double)imvec_[i][e]-peak_I);
                    }

                    for (int e = C+1; e <= B; e++)
                    {
                        R(e-C-1,0)=e-cog_x1;
                        br(e-C-1,0)=((double)imvec_[i][e]-peak_I);
                    }

                    Eigen::VectorXd xl=L.householderQr().solve(bl);
                    Eigen::VectorXd xr=R.householderQr().solve(br);
                    double lslope=xl(0);
                    double rslope=xr(0);

                    if(lslope<=0||rslope>=0) continue;

                    Eigen::Vector2d vl(1,lslope),vr(1,rslope);
                    double alpha=M_PI-acos(vl.dot(vr)/(vl.norm()*vr.norm()));                    

                    if(alpha>peak_sharpness) continue;

                    std::vector<int> laser_pt(2);
                    laser_pt[0]=cog_x0;
                    laser_pt[1]=i;
                    const std::lock_guard<std::mutex> lock(mtx);
                    laser_pts.push_back(laser_pt);
                }
            }
        };

        tpool.push_back(std::thread(fn));
    }
    
    for (auto& t:tpool)
    {
        t.join();
    }

    return laser_pts;
}