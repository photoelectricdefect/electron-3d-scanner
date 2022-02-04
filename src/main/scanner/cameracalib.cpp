#include <cameracalib.hpp>
#include <fstream>
#include <iostream>

namespace scanner {
    cameracalib::cameracalib() {};

void to_json(nlohmann::json& j, const jcameracalib& data) {
    j = nlohmann::json{{ "K", data.K }, 
                            { "D", data.D }};
}

void from_json(const nlohmann::json& j, jcameracalib& data) {
    j.at("K").get_to(data.K);
    j.at("D").get_to(data.D);
}

void cameracalib::save(std::string fpath) {
    std::vector<double> K_, D_;
    for(int i = 0; i < K.rows; i++) {
        for(int j = 0; j < K.cols; j++) {
            K_.push_back(K.ptr<double>(i)[j]);
        }
    }

    for(int i = 0; i < D.rows; i++) {
        for(int j = 0; j < D.cols; j++) {
            D_.push_back(D.ptr<double>(i)[j]);
        }
    }

    jcameracalib data={K_,D_};
    nlohmann::json j=data;
    std::ofstream file;
    file.open(fpath);
    file << j.dump();
    file.close();
}       

bool cameracalib::load(std::string fpath) {
    try {
    std::ifstream file(fpath);
    nlohmann::json j;
    file >> j;
    file.close();
        jcameracalib data = j.get<jcameracalib>();
    K = cv::Mat(cv::Size(3, 3), CV_64FC1);
    D = cv::Mat(cv::Size(1, 5), CV_64FC1);

    for(int i = 0; i < K.rows; i++) {
        for(int j = 0; j < K.cols; j++) {
            K.ptr<double>(i)[j] = data.K[i * K.cols + j];
        }
    }

    for(int i = 0; i < D.rows; i++) {
        for(int j = 0; j < D.cols; j++) {
            D.ptr<double>(i)[j] = data.D[i * D.cols + j];            
        }
    }

    return true;

    } catch(nlohmann::json::parse_error& e) {
        std::cerr<<e.what()<<std::endl;
    }

    return false;
    }
}
