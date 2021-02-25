#include <scannercalib.hpp>
#include <fstream>
#include <iostream>

namespace scanner {
    scannercalib::scannercalib() {};

    void to_json(nlohmann::json& j, const jscannercalib& data) {
        j = nlohmann::json{ {"laser_plane", data.laser_plane}};
    }

    void from_json(const nlohmann::json& j, jscannercalib& data) {
        j.at("laser_plane").get_to(data.laser_plane);
    }

void scannercalib::save(std::string fpath) {
    std::vector<double> laser_plane_;
    jscannercalib data = {laser_plane_};
    nlohmann::json j = data;
    std::ofstream file;
    file.open(fpath);
    file << j.dump();
    file.close();
}

bool scannercalib::load(std::string fpath) {
        try {
    std::ifstream file(fpath);
    nlohmann::json j;
    file >> j;
    file.close();
    jscannercalib data = j.get<jscannercalib>();
    Eigen::Vector3d n;

    for(int i = 0; i < 3; i++) {
        n<<data.laser_plane[i];
    }

    laser_plane=Eigen::Hyperplane<double,3>(n,data.laser_plane[3]);

    return true;
    } catch(nlohmann::json::parse_error& e) {
        std::cerr<<e.what()<<std::endl;
    }

    return false;
    }
}
