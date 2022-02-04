#include <scannercalib.hpp>
#include <fstream>
#include <iostream>

namespace scanner {
    scannercalib::scannercalib() {};

    void to_json(nlohmann::json& j, const jscannercalib& data) {
        j = nlohmann::json{ {"laser_plane", data.laser_plane},{ "rotation_axis_direction", data.rotation_axis_direction },{ "rotation_axis_source", data.rotation_axis_source }/*{ "RT_rotation_axis", data.RT_rotation_axis }*/};
    }

    void from_json(const nlohmann::json& j, jscannercalib& data) {
        j.at("laser_plane").get_to(data.laser_plane);
        j.at("rotation_axis_direction").get_to(data.rotation_axis_direction);
        j.at("rotation_axis_source").get_to(data.rotation_axis_source);

        // j.at("RT_rotation_axis").get_to(data.RT_rotation_axis);
    }

    void to_json(nlohmann::json& j, const jpoints& data) {
        j = nlohmann::json{ {"direction", data.direction},{ "source", data.source },{ "orbit_points", data.orbit_points },{ "center_points", data.center_points },{ "npoints", data.npoints }};
    }

    void from_json(const nlohmann::json& j, jpoints& data) {
        j.at("direction").get_to(data.direction);
        j.at("source").get_to(data.source);
        j.at("orbit_points").get_to(data.orbit_points);
        j.at("center_points").get_to(data.center_points);
        j.at("npoints").get_to(data.npoints);
    }

Eigen::Matrix4d scannercalib::get_axis_rigid_body_transform() {
    Eigen::Vector3d y_axis=rotation_axis_direction;
    Eigen::Vector3d x_axis(y_axis(1),-y_axis(0),0);
    Eigen::Vector3d z_axis=y_axis.cross(x_axis);
    x_axis.normalize(),y_axis.normalize(),z_axis.normalize();    
    Eigen::Matrix4d RT;
    RT.block<1,3>(3,0).setZero();
    RT.block<3,1>(0,0)=x_axis;
    RT.block<3,1>(0,1)=y_axis;
    RT.block<3,1>(0,2)=z_axis;
    RT.block<3,1>(0,3)=rotation_axis_origin;
    RT(3,3)=1;

    return RT;
}

void scannercalib::save_points(std::string fpath,const std::vector<double>& direction,const std::vector<double>& source,const std::vector<std::vector<double>>& orbit_points,const std::vector<std::vector<double>>& center_points,int npoints) {
    jpoints data = {direction,source,orbit_points,center_points,npoints};
    nlohmann::json j = data;
    std::ofstream file;
    file.open(fpath);
    file << j.dump();
    file.close();
}

void scannercalib::load_points(std::string fpath,std::vector<double>& direction,std::vector<double>& source,std::vector<std::vector<double>>& orbit_points,std::vector<std::vector<double>>& center_points,int& npoints) {
    std::ifstream file(fpath);
    nlohmann::json j;
    file >> j;    
    file.close();
    auto data = j.get<jpoints>();
    npoints=data.npoints;

    for(int i = 0; i < data.direction.size(); i++) {
        direction.push_back(data.direction[i]);
    }

    for(int i = 0; i < data.source.size(); i++) {
        source.push_back(data.source[i]);
    }

    orbit_points.push_back(std::vector<double>());
    orbit_points.push_back(std::vector<double>());
    orbit_points.push_back(std::vector<double>());

    for(int i = 0; i < data.orbit_points[0].size(); i++) {
        orbit_points[0].push_back(data.orbit_points[0][i]);
        orbit_points[1].push_back(data.orbit_points[1][i]);
        orbit_points[2].push_back(data.orbit_points[2][i]);
    }

    center_points.push_back(std::vector<double>());
    center_points.push_back(std::vector<double>());
    center_points.push_back(std::vector<double>());

    for(int i = 0; i < data.center_points[0].size(); i++) {
        center_points[0].push_back(data.center_points[0][i]);
        center_points[1].push_back(data.center_points[1][i]);
        center_points[2].push_back(data.center_points[2][i]);
    }
}

void scannercalib::save(std::string fpath) {
    std::vector<double> laser_plane_vec(4);
    auto coeffs=laser_plane.coeffs();
    laser_plane_vec[0]=coeffs(0),laser_plane_vec[1]=coeffs(1),
    laser_plane_vec[2]=coeffs(2),laser_plane_vec[3]=coeffs(3);
    
    // std::vector<double> RT;
    // for(int i = 0; i < RT_rotation_axis.rows(); i++) {
    //     for(int j = 0; j < RT_rotation_axis.cols(); j++) {
    //         RT.push_back(RT_rotation_axis(i,j));
    //     }
    // }

    std::vector<double> direction_vec;
    direction_vec.push_back(rotation_axis_direction(0));
    direction_vec.push_back(rotation_axis_direction(1));
    direction_vec.push_back(rotation_axis_direction(2));

    std::vector<double> source_vec;
    source_vec.push_back(rotation_axis_origin(0));
    source_vec.push_back(rotation_axis_origin(1));
    source_vec.push_back(rotation_axis_origin(2));

    jscannercalib data = {laser_plane_vec,direction_vec,source_vec};
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
            n(i)=data.laser_plane[i];
        }

        laser_plane=Eigen::Hyperplane<double,3>(n,data.laser_plane[3]);
        rotation_axis_direction(0)=data.rotation_axis_direction[0];
        rotation_axis_direction(1)=data.rotation_axis_direction[1];
        rotation_axis_direction(2)=data.rotation_axis_direction[2];
        rotation_axis_origin(0)=data.rotation_axis_source[0];
        rotation_axis_origin(1)=data.rotation_axis_source[1];
        rotation_axis_origin(2)=data.rotation_axis_source[2];

        // for(int i = 0; i < RT_rotation_axis.rows(); i++) {
        //     for(int j = 0; j < RT_rotation_axis.cols(); j++) {
        //         RT_rotation_axis(i,j) = data.RT_rotation_axis[i * RT_rotation_axis.cols() + j];            
        //     }
        // }

        return true;
        }   
        catch(nlohmann::json::parse_error& e) {
            std::cerr<<e.what()<<std::endl;
        }

        return false;
    }
}
