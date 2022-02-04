#include <helpers/cv_helpers.hpp>
#include <base64.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

namespace cv_helpers {

void set_px(cv::Mat& img, int x, int y, uint8_t intensity) {
	img.ptr<uint8_t>(y)[x] = intensity;
}

void set_px3C(cv::Mat& img, int x, int y, cv::Vec3b intensity) {
	img.ptr<cv::Vec3b>(y)[x] = intensity;
}

uint8_t get_px(const cv::Mat& img, int x, int y) {
	return img.ptr<uint8_t>(y)[x];
}

cv::Vec3b get_px3C(const cv::Mat& img, int x, int y) {
	return img.ptr<cv::Vec3b>(y)[x];
}

void sharpen(const cv::Mat& img, const cv::Mat& sharpened, float alpha, int threshold) {
	cv::GaussianBlur(img, sharpened, cv::Size(5,5),0);
	cv::addWeighted(img,alpha,sharpened,1-alpha,0,sharpened);
	cv::Mat low_contrast_mask = cv::abs(img - sharpened) < threshold;
	img.copyTo(sharpened, low_contrast_mask);
}

//FIX
// float std_dev(cv::Mat m) {
// 	cv::Mat m_blurred, m_squared_blurred;
// 	cv::blur(m, m_blurred, cv::Size(3, 3)), cv::blur(m.mul(m), m_squared_blurred, cv::Size(3, 3));
// 	float sigma = cv::sqrt(m_squared_blurred - m_blurred.mul(m_blurred));

// 	return sigma;
// }

double cross2D(const Eigen::Vector2d& u, const Eigen::Vector2d& v)
{
	return u(0) * v(1) - v(0) * u(1);
}

bool inside_polygon(const Eigen::Vector2d& a, const std::vector<line_segment>& sides) {
			int intersections = 0;
			Eigen::Vector2d O(0, a(1));
			bool on_line = false;

			for(int i = 0; i < sides.size(); i++) {
				Eigen::Vector2d pq = O - sides[i].a,
				r = sides[i].b - sides[i].a,
				s = a - O;
				double e = cross2D(pq, r),
				b = cross2D(pq, s),
				c = cross2D(r, s);

				if (c != 0)
				{
					double t = b / c,
					u = e / c;

					if (0 <= t && t <= 1 &&
					0 <= u && u <= 1)
					{
						intersections++;

						if(u == 1)  {
							on_line = true;
							break;
						}
					}
				}
			}	

			if(intersections % 2 > 0 && !on_line) return true;
			return false;
}


void cut(cv::Mat& mask, cv::Size img_size, const std::vector<line_segment>& borders) {
	mask = cv::Mat(img_size, CV_8UC1);

	for(int i = 0; i < img_size.height; i++) {
		for(int j = 0; j < img_size.width; j++) {
			bool inside = inside_polygon(Eigen::Vector2d(j, i), borders);

			if(!inside) set_px(mask, j, i, MIN_INTENSITY);
			else set_px(mask, j, i, MAX_INTENSITY);
		}
	}
}

void crop(const cv::Mat& img,cv::Mat& cropped,const Eigen::Vector2d& q,const Eigen::Vector2d& s) {	
	cv::Rect ROI(q(0), q(1), abs(s(0) - q(0)), abs(s(1) - q(1)));
	cropped = img(ROI);
}

float masked_threshold(cv::Mat& img, const cv::Mat& mask, int type, int threshold) {
	int idx = 0,
	n_px = cv::countNonZero(mask);
	cv::Mat tmp(cv::Size(n_px, 1), CV_8UC1);
	
	for(int c = 0; c < mask.size().height; c++) {
		for(int e = 0; e < mask.size().width; e++) {
			if(get_px(mask, e, c) == MAX_INTENSITY) {
				set_px(tmp, idx, 0, get_px(img, e, c));
				idx++;
			} 
		}
	}

	float t = cv::threshold(tmp, tmp, threshold, 255, type);
	idx = 0;

	for(int c = 0; c < mask.size().height; c++) {
		for(int e = 0; e < mask.size().width; e++) {
			if(get_px(mask, e, c) == MAX_INTENSITY) {
				set_px(img, e, c, get_px(tmp, idx, 0));
				idx++;
			} 
		}
	}

	return t;
}

void ZhangSuen_thinning(const cv::Mat& binary, cv::Mat& out) {
	const int dx[9]={0,1,1,1,0,-1,-1,-1,0};
	const int dy[9]={-1,-1,0,1,1,1,0,-1,-1};
	out=binary.clone();
	int changed=1;

	while(changed>0) {
		changed=0;

		cv::Mat out_tmp=out.clone();

	for (size_t x = 0; x < out.cols; x++)
	{
		for (size_t y=0;y<out.rows;y++)
		{
			bool cond0,cond1;
			cond0=cond1=false;
			int transitions=0;
			int whites=0;
			uint8_t C=get_px(out,x,y);

			if(x==0||x==out.cols-1||y==0||y==out.rows-1||C==MIN_INTENSITY)  {
				continue;
			}

				for (size_t i = 0; i < 8; i++)
				{
					int x0=x+dx[i];
					int y0=y+dy[i];
					int x1=x+dx[i+1];
					int y1=y+dy[i+1];
					uint8_t C0=get_px(out,x0,y0);
					uint8_t C1=get_px(out,x1,y1);

					if((i==0||i==2||i==4)&&C0==MIN_INTENSITY) cond0=true;
					if((i==2||i==4||i==6)&&C0==MIN_INTENSITY) cond1=true;

					if(C0==MAX_INTENSITY) whites++;
					else if(C1==MAX_INTENSITY) transitions++;
				}

			if(whites>=2&&whites<=6&&transitions==1&&cond0&&cond1) {
				set_px(out_tmp,x,y,MIN_INTENSITY);
				changed++;
			}
		}
	}

	out=out_tmp.clone();

	for (size_t x = 0; x < out.cols; x++)
	{
		for (size_t y=0;y<out.rows;y++)
		{
			bool cond0,cond1;
			cond0=cond1=false;
			int transitions=0;
			int whites=0;
			uint8_t C=get_px(out_tmp,x,y);

			if(x==0||x==out.cols-1||y==0||y==out.rows-1||C==MIN_INTENSITY)  {
				continue;
			}

				for (size_t i = 0; i < 8; i++)
				{
					int x0=x+dx[i];
					int y0=y+dy[i];
					int x1=x+dx[i+1];
					int y1=y+dy[i+1];
					uint8_t C0=get_px(out_tmp,x0,y0);
					uint8_t C1=get_px(out_tmp,x1,y1);

					if((i==0||i==2||i==6)&&C0==MIN_INTENSITY) cond0=true;
					if((i==0||i==4||i==6)&&C0==MIN_INTENSITY) cond1=true;

					if(C0==MAX_INTENSITY) whites++;
					else if(C1==MAX_INTENSITY) transitions++;
				}

			if(whites>=2&&whites<=6&&transitions==1&&cond0&&cond1) {
				set_px(out,x,y,MIN_INTENSITY);
				changed++;
			}
		}
		}
	}
}

void PCA(cv::Mat& data, Eigen::MatrixXd& V, Eigen::MatrixXd& D, Eigen::Vector2d& O) {
	cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> v(pca.eigenvectors.ptr<double>(), data.cols, data.cols);
	V = v;
	O = Eigen::Vector2d(pca.mean.ptr<double>(0)[0], pca.mean.ptr<double>(0)[1]);
	D = Eigen::MatrixXd(data.cols, data.cols);

	for(int i = 0; i < data.cols; i++) {		
		D(i, i) = *pca.eigenvalues.ptr<double>(i);
	}
}

size_t mat2buffer(cv::Mat& img, uint8_t*& data) {
    std::vector<uint8_t> buf;
    cv::imencode(".jpg", img, buf);
	data = new uint8_t[buf.size()];
	std::copy(buf.begin(), buf.end(), data);

    return buf.size();
}

std::string mat2base64str(cv::Mat& img) {
    std::vector<uint8_t> buf;
    cv::imencode(".jpg", img, buf);
    auto *enc = reinterpret_cast<unsigned char*>(buf.data());
    return base64_encode(enc, buf.size());
}

size_t mat2base64(cv::Mat& img, char*& data) {
    auto str = mat2base64str(img);
	data = new char[str.length()];
	std::copy( str.begin(), str.end(), data );
	// std::cout << data << std::endl;
	return str.length();
}

// void Bradley_thresholding(const cv::Mat& gray,cv::Mat& out,int wsize,int t,int thresh_min) {
//     cv::Mat integral_image=cv::Mat::zeros(gray.size(),CV_32SC1);
//     cv::Mat integral_image_sq=cv::Mat::zeros(gray.size(),CV_32SC1);
//     cv::Mat integral_stdev=cv::Mat::zeros(gray.size(),CV_32SC1);

// 	cv::Mat gray_16i=cv::Mat::zeros(gray.size(),CV_16SC1),gray_sq;
// 	gray.convertTo(gray_16i,CV_16SC1);
// 	gray_sq=gray_16i.mul(gray_16i);

// 	std::cout<<"imagew:"<<gray_sq.cols<<std::endl;
// 	std::cout<<"imageh:"<<gray_sq.rows<<std::endl;

// 	summed_area_table<uint8_t,int>(gray,integral_image);
// 	summed_area_table<int,int>(gray_sq,integral_image_sq);

//     int d=wsize/2;

//     for(size_t x=0;x<gray.cols;x++) {
//         for(size_t y=0;y<gray.rows;y++) {
//             int xbr=x+d,ybr=y+d;
//             int xtl=x-d-1,ytl=y-d-1;
//             int xtr=x+d,ytr=y-d-1;
//             int xbl=x-d-1,ybl=y+d;

//             if(xbr>=gray.cols) xbr=gray.cols-1;
//             if(ybr>=gray.rows) ybr=gray.rows-1;
            
//             int br=integral_image.ptr<int>(ybr)[xbr];
//             int br_sq=integral_image_sq.ptr<int>(ybr)[xbr];
//             int winsum=br,winsum_sq=br_sq;

//             if(xtl>=0&&ytl>=0) {
//                 int tl=integral_image.ptr<int>(ytl)[xtl];
//                 int tl_sq=integral_image_sq.ptr<int>(ytl)[xtl];

//                 winsum+=tl;
//                 winsum_sq+=tl_sq;
//             }

//             if(xbl>=0) {
//                 if(ybl>=gray.rows) ybl=gray.rows-1;

//                 int bl=integral_image.ptr<int>(ybl)[xbl];
//                 winsum-=bl;
//                 int bl_sq=integral_image_sq.ptr<int>(ybl)[xbl];
//                 winsum_sq-=bl_sq;

//             }

//             if(ytr>=0) {
//                 if(xtr>=gray.cols) xtr=gray.cols-1;

//                 int tr=integral_image.ptr<int>(ytr)[xtr];
//                 winsum-=tr;
//                 int tr_sq=integral_image_sq.ptr<int>(ytr)[xtr];
//                 winsum_sq-=tr_sq;

//             }

//             int wtlx=x-d,wtly=y-d,wbrx=xbr,wbry=ybr;
            
// 			if(wtlx<0) wtlx=0;
// 			if(wtly<0) wtly=0;

// 			uint8_t C=get_px1<uint8_t>(gray,x,y);
//             int npx=(wbrx-wtlx+1)*(wbry-wtly+1);
//             int predicted_sum=npx*(int)C;
// 			double variance=(winsum_sq-pow(winsum,2)/npx)/npx;
// 			double stdev=sqrt(variance);

// 			if(x==100&&y==100) {
// 				std::cout<<"var: "<<variance<<std::endl;
// 				std::cout<<"stddev: "<<stdev<<std::endl;
// 				std::cout<<"wsq: "<<winsum_sq<<std::endl;
// 				std::cout<<"ws: "<<winsum<<std::endl;

// 			}

//             if(predicted_sum<=winsum*(100-t)/100.f||C<=thresh_min) set_px1<uint8_t>(out,x,y,0);
// 			else set_px1<uint8_t>(out,x,y,255);
//         }
//     }
// }

// //Should keep
// void find_lines(cv::Mat img, std::vector<cv::Vec2f >& lines, float r, float theta, int hough_threshold, int low_treshold, int high_threshold, int kernel_size)
// {
// 	cv::Mat edges;
// 	cv::Canny(img, edges, low_treshold, high_threshold, kernel_size);
//     cv::imshow("edges", edges);
// 	cv::HoughLines(edges, lines, r, theta, hough_threshold, 0, 0);
// }

// //Should keep??
// void draw_lines(cv::Mat img, std::vector<cv::Vec2f > lines, cv::Scalar clr, int width)
// {
// 	int diag = round(sqrt(pow(img.size().height, 2) + pow(img.size().width, 2)));

// 	for (auto &line: lines)
// 	{
// 		float r = line[0];
// 		float sin_theta = sin(line[1]);
// 		float cos_theta = cos(line[1]);
// 		int x0 = round(r *cos_theta);
// 		int y0 = round(r *sin_theta);
// 		cv::Point a, b;
// 		a.x = cvRound(x0 + diag *sin_theta);
// 		a.y = cvRound(y0 - diag *cos_theta);
// 		b.x = cvRound(x0 - diag *sin_theta);
// 		b.y = cvRound(y0 + diag *cos_theta);
// 		cv::line(img, a, b, clr, width, cv::LINE_AA);
// 	}
// }

// //Should keep??
// void triangulation_matting(cv::Mat B1, cv::Mat I1, cv::Mat B2, cv::Mat I2, cv::Mat& alpha) {
// 	alpha = cv::Mat(I1.size(), CV_8UC1);
// 	Eigen::Matrix<float, 6, 4> A;
// 	A << 1, 0, 0, 0,
// 		0, 1, 0, 0,
// 		0, 0, 1, 0,
// 		1, 0, 0, 0,
// 		0, 1, 0, 0,
// 		0, 0, 1, 0;
// 	Eigen::Matrix<float, 6, 1> b;
// 	Eigen::Matrix<float, 4, 1> x;

// 	for(int i = 0; i < B1.rows; i++) {
// 		for(int j = 0; j < B1.cols; j++) {
// 			cv::Vec3b I1_px = get_px_3C(I1, j, i);
// 			cv::Vec3b B1_px = get_px_3C(B1, j, i);
// 			cv::Vec3b I2_px = get_px_3C(I2, j, i);
// 			cv::Vec3b B2_px = get_px_3C(B2, j, i);
// 			b(0) = I1_px(0) - B1_px(0);
// 			b(1) = I1_px(1) - B1_px(1);
// 			b(2) = I1_px(2) - B1_px(2);
// 			b(3) = I2_px(0) - B2_px(0);
// 			b(4) = I2_px(1) - B2_px(1);
// 			b(5) = I2_px(2) - B2_px(2);
// 			A(0, 3) = -B1_px(0);	
// 			A(1, 3) = -B1_px(1);
// 			A(2, 3) = -B1_px(2);
// 			A(3, 3) = -B2_px(0);
// 			A(4, 3) = -B2_px(1);
// 			A(5, 3) = -B2_px(2);	
// 			x = A.colPivHouseholderQr().solve(b);

// 			// // x(3) = abs(x(3));
// 			if (x(0) < 0) { x(0) = 0; }
// 			else if (x(0) > 255) { x(0) = 255; }
						
// 			if (x(1) < 0) { x(1) = 0; }
// 			else if (x(1) > 255) { x(1) = 255; }
						
// 			if (x(2) < 0) { x(2) = 0; }
// 			else if (x(2) > 255) { x(2) = 255; }
						
// 			if (x(3) < 0) { x(3) = 0; }
// 			else if (x(3) > 1) { x(3) = 1; }

// 			set_px(alpha, j, i, x(3) * MAX_INTENSITY);
// 		}

// 	}
// }

// //Should keep??
// void draw_lines(cv::Mat img, std::vector<line_segment> lines, cv::Scalar clr, int width)
// {
// 	for (auto &ln: lines)
// 	{
// 		cv::line(img, cv::Point(ln.a(0), ln.a(1)), cv::Point(ln.b(0), ln.b(1)), clr, width, cv::LINE_AA);
// 	}
// }

// //Should keep??
// void SLIC(cv::Mat img, cv::Mat& dst, cv::ximgproc::SLICType type, int size, int connectivity) {
// 	cv::Ptr<cv::ximgproc::SuperpixelSLIC> SLIC = cv::ximgproc::createSuperpixelSLIC(img, type, size, connectivity);
// 	SLIC->iterate();
// 	SLIC->enforceLabelConnectivity(50);
// 	cv::Mat mask;
// 	SLIC->getLabelContourMask(mask, true);
// 	dst.setTo(cv::Scalar(0, 255, 0), mask);
// }
}