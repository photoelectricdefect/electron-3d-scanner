#include <cameracalib.hpp>
#include <fstream>

namespace scanner {
    cameracalib::cameracalib() {};

    void to_json(nlohmann::json& j, const jcameracalib& data) {
        j = nlohmann::json{{ "ncaps", data.ncaps }, { "K", data.K }, 
                            { "D", data.D },
                            {"board_size", data.board_size}, 
                            {"square_size", data.square_size}};
    }

    void from_json(const nlohmann::json& j, jcameracalib& data) {
        j.at("ncaps").get_to(data.ncaps),
        j.at("K").get_to(data.K),
        j.at("D").get_to(data.D),
        j.at("board_size").get_to(data.board_size),
        j.at("square_size").get_to(data.square_size);

    }

    //TODO: read configuration and saved data from files
    void cameracalib::load() {
        board_size = cv::Size(9, 6);
        square_size = cv::Size(23, 23);
        ncaps = 20;
    }

void cameracalib::save(std::string fpath) {
    std::vector<double> K_, D_, square_size_;
    std::vector<int> board_size_;

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

    board_size_.push_back(board_size.width), 
    board_size_.push_back(board_size.height),
    square_size_.push_back(square_size.width),
    square_size_.push_back(square_size.height);

    jcameracalib data = {ncaps, K_, D_, square_size_, board_size_};
    nlohmann::json j = data;
    std::ofstream file;
    file.open(fpath);
    file << j.dump();
    file.close();
}

//For now manually init 
    // void /*cameracalib::*/load(std::string fpath) {
    // std::ifstream file(fpath);
    // nlohmann::json j;
    // file >> j;
    // file.close();
    // jcameracalib data = j.get<jcameracalib>();

    // //Handle no existing save data
    // //if(data.K.size() == 0 || data.D.size() == 0) return false;

    // ncaps = data.ncaps;
    // board_size = cv::Size(data.board_size[0], data.board_size[1]);
    // square_size = cv::Size(data.square_size[0], data.square_size[1]);
    // K = cv::Mat(cv::Size(3, 3), CV_64FC1);
    // D = cv::Mat(cv::Size(1, 4), CV_64FC1);

    // for(int i = 0; i < K.rows; i++) {
    //     for(int j = 0; j < K.cols; j++) {
    //         K.ptr<double>(i)[j] = data.K[i * K.cols + j];
    //     }
    // }

    // for(int i = 0; i < D.rows; i++) {
    //     for(int j = 0; j < D.cols; j++) {
    //         D.ptr<double>(i)[j] = data.D[i * D.cols + j];            
    //     }
    // }

    // }
}
