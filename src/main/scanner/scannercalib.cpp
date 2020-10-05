#include <scannercalib.hpp>
#include <fstream>

namespace scanner {
    scannercalib::scannercalib() {};

    //TODO: read configuration and saved data from files
    void scannercalib::load() {
        board_size = cv::Size(9, 6);
        square_size = cv::Size(23, 23);
        //ncaps = 20;
    }

    void to_json(nlohmann::json& j, const jscannercalib& data) {
        j = nlohmann::json{ {"board_size", data.board_size}, 
                            {"square_size", data.square_size}};
    }

    void from_json(const nlohmann::json& j, jscannercalib& data) {
        j.at("board_size").get_to(data.board_size),
        j.at("square_size").get_to(data.square_size);

    }

void scannercalib::save(std::string fpath) {
    std::vector<double> square_size_;
    std::vector<int> board_size_;

    board_size_.push_back(board_size.width), 
    board_size_.push_back(board_size.height),
    square_size_.push_back(square_size.width),
    square_size_.push_back(square_size.height);

    jscannercalib data = {square_size_, board_size_};
    nlohmann::json j = data;
    std::ofstream file;
    file.open(fpath);
    file << j.dump();
    file.close();
}

//For now manually init 
    // void /*scannercalib::*/load(std::string fpath) {
    // std::ifstream file(fpath);
    // nlohmann::json j;
    // file >> j;
    // file.close();
    // jscannercalib data = j.get<jscannercalib>();

    // //Handle no existing save data

    // board_size = cv::Size(data.board_size[0], data.board_size[1]);
    // square_size = cv::Size(data.square_size[0], data.square_size[1]);
    // }
}
