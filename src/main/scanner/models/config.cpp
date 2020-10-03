#include <models/config.hpp>


//TODO: move to cameracalib
namespace scanner {
    void to_json(nlohmann::json& j, const calib_data& data) {
        j = nlohmann::json{{ "K", data.K }, { "D", data.D }};
    }

    void from_json(const nlohmann::json& j, calib_data& data) {
        j.at("K").get_to(data.K);
        j.at("D").get_to(data.D);
    }


void config::save_calib(const cv::Mat& K, const cv::Mat& D) {
    std::vector<double> _K, _D;

    for(int i = 0; i < K.rows; i++) {
        for(int j = 0; j < K.cols; j++) {
            _K.push_back(K.ptr<double>(i)[j]);
        }
    }

    for(int i = 0; i < D.rows; i++) {
        for(int j = 0; j < D.cols; j++) {
            _D.push_back(D.ptr<double>(i)[j]);
        }
    }

    calib_data data = {_K, _D};
    nlohmann::json j = data;
    std::ofstream file;
    file.open("calib.json");
    file << j.dump();
    file.close();
}

void config::load_calib(cv::Mat& K, cv::Mat& D) {
    std::ifstream file("calib.json");
    nlohmann::json j;
    file >> j;
    calib_data data = j.get<calib_data>();

    //Handle no existing save data
    //if(data.K.size() == 0 || data.D.size() == 0) return false;

    K = cv::Mat(cv::Size(3, 3), CV_64FC1);
    D = cv::Mat(cv::Size(1, 4), CV_64FC1);

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

    }
}