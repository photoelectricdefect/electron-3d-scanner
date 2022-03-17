#include <camera_calibration.hpp>
#include <fstream>
#include <iostream>

namespace scanner {
    camera_calibration::camera_calibration() {};

void camera_calibration::create_calibration_file(const std::string& fpath) {
    std::ofstream file(fpath);
    
    if(!file.is_open())
    {
        std::cerr<<"could not open camera calibration file for writing"<<std::endl;
        return;
    }
    
    nlohmann::json j;
    j["pattern_size"]=std::vector<int>(2,0);
    j["n_captures"]=0;
    j["square_size"]=0;
    file<<j.dump(1);
    file.close();
}

void camera_calibration::save(const std::string& fpath) {
    nlohmann::json j;
    std::fstream file(fpath,std::ios::in|std::ios::out);
    
    if(!file.is_open())
    {
        std::cerr<<"could not open camera calibration file for reading and writing"<<std::endl;
        return;
    }

    file>>j;
    std::vector<double> K_vector, D_vector;
    
    for(int i = 0; i < K.rows; i++) {
        for(int j = 0; j < K.cols; j++) {
            K_vector.push_back(K.ptr<double>(i)[j]);
        }
    }

    for(int i = 0; i < D.rows; i++) {
        for(int j = 0; j < D.cols; j++) {
            D_vector.push_back(D.ptr<double>(i)[j]);
        }
    }

    j["K"]=K_vector;
    j["D"]=D_vector;
    file << j.dump(1);
    file.close();
}       

void camera_calibration::load(const std::string& fpath) {
        std::ifstream file(fpath);
        
        if(!file.is_open())
        {
            std::cerr<<"could not open camera calibration file for reading"<<std::endl;
            return;
        }

        nlohmann::json j;
        file >> j;
        file.close();
    
        for (auto& item : j.items()) {
            if(!item.key().compare("K")) {
                auto K_vector=item.value().get<std::vector<double>>();
                K = cv::Mat(cv::Size(3, 3), CV_64FC1);

                for(int i = 0; i < K.rows; i++) {
                    for(int j = 0; j < K.cols; j++) {
                        K.ptr<double>(i)[j] = K_vector[i * K.cols + j];
                    }
                }
            }
            else if(!item.key().compare("D")) {
                auto D_vector=item.value().get<std::vector<double>>();
                D = cv::Mat(cv::Size(1, 5), CV_64FC1);

                for(int i = 0; i < D.rows; i++) {
                    for(int j = 0; j < D.cols; j++) {
                        D.ptr<double>(i)[j] = D_vector[i * D.cols + j];            
                    }
                }
            }
            else if(!item.key().compare("square_size")) {
                square_size=item.value().get<double>();
            }
            else if(!item.key().compare("n_captures")) {
                n_captures=item.value().get<int>();
            }
            else if(!item.key().compare("pattern_size")) {
                auto pattern_size_vector=item.value().get<std::vector<int>>();
                pattern_size=cv::Size(pattern_size_vector[0],pattern_size_vector[1]);
            }
        }
    }    
}
