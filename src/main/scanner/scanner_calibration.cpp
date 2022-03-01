#include <scanner_calibration.hpp>
#include <fstream>
#include <iostream>

namespace scanner {
    scanner_calibration::scanner_calibration() {};

Eigen::Matrix4d scanner_calibration::get_axis_rigid_body_transform() {
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

void scanner_calibration::save_points(std::string fpath,const std::vector<double>& direction,const std::vector<double>& source,const std::vector<std::vector<double>>& orbit_points,const std::vector<std::vector<double>>& center_points,int npoints) {
    jpoints data = {direction,source,orbit_points,center_points,npoints};
    nlohmann::json j = data;
    std::ofstream file;
    file.open(fpath);
    file << j.dump();
    file.close();
}

void scanner_calibration::load_points(std::string fpath,std::vector<double>& direction,std::vector<double>& source,std::vector<std::vector<double>>& orbit_points,std::vector<std::vector<double>>& center_points,int& npoints) {
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

void scanner_calibration::create_calibration_file(std::string fpath) {
    std::ofstream file(fpath);
    
    if(!file.is_open())
    {
        std::cerr<<"could not open camera calibration file for writing"<<std::endl;
        return;
    }
    
    nlohmann::json j;
    j["pattern_size"]=std::vector<int>(2,0);
    j["square_size"]=0;
    j["rotation_axis_radius"]=0;
    j["n_calibration_images"]=0;
    j["stepper_gear_ratio"]=0;
    j["steps_per_calibration_image"]=0;
    j["n_captures"]=0;
    file<<j.dump(1);
    file.close();
}

void scanner_calibration::save(std::string fpath) {
    nlohmann::json j;
    std::fstream file(fpath,std::ios::in|std::ios::out);
    
    if(!file.is_open())
    {
        std::cerr<<"could not open camera calibration file for reading and writing"<<std::endl;
        return;
    }

    file>>j;
    std::vector<double> laser_plane_vector(4),rotation_axis_direction_vector(3),rotation_axis_origin_vector(3);
    auto coeffs=laser_plane.coeffs();
    laser_plane_vector[0]=coeffs(0),laser_plane_vector[1]=coeffs(1),
    laser_plane_vector[2]=coeffs(2),laser_plane_vector[3]=coeffs(3);
    
    rotation_axis_direction_vector[0]=rotation_axis_direction(0);
    rotation_axis_direction_vector[1]=rotation_axis_direction(1);
    rotation_axis_direction_vector[2]=rotation_axis_direction(2);

    rotation_axis_origin_vector[0]=rotation_axis_origin(0);
    rotation_axis_origin_vector[1]=rotation_axis_origin(1);
    rotation_axis_origin_vector[2]=rotation_axis_origin(2);

    j["laser_plane"]=laser_plane_vector;
    j["rotation_axis_origin"]=rotation_axis_origin_vector;
    j["rotation_axis_direction"]=rotation_axis_direction_vector;
    
    file << j.dump(1);
    file.close();
}

void scanner_calibration::load(std::string fpath) {
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
            if(!item.key().compare("laser_plane")) {
                auto laser_plane_vector=item.value().get<std::vector<double>>();
                Eigen::Vector3d n(laser_plane_vector[0],laser_plane_vector[1],laser_plane_vector[2]);
                laser_plane=Eigen::Hyperplane<double,3>(n,laser_plane_vector[3]);
            }
            else if(!item.key().compare("rotation_axis_direction")) {
                auto rotation_axis_direction_vector=item.value().get<std::vector<double>>();
                rotation_axis_direction=Eigen::Vector3d(rotation_axis_direction_vector[0],rotation_axis_direction_vector[1],rotation_axis_direction_vector[2]);
            }
            else if(!item.key().compare("rotation_axis_origin")) {
                auto rotation_axis_origin_vector=item.value().get<std::vector<double>>();
                rotation_axis_origin=Eigen::Vector3d(rotation_axis_origin_vector[0],rotation_axis_origin_vector[1],rotation_axis_origin_vector[2]);
            }
            else if(!item.key().compare("square_size")) {
                square_size=item.value().get<double>();
            }
            else if(!item.key().compare("rotation_axis_radius")) {
                rotation_axis_radius=item.value().get<double>();
            }
            else if(!item.key().compare("n_calibration_images")) {
                n_calibration_images=item.value().get<int>();
            }
            else if(!item.key().compare("steps_per_calibration_image")) {
                steps_per_calibration_image=item.value().get<int>();
            }
            else if(!item.key().compare("stepper_gear_ratio")) {
                stepper_gear_ratio=item.value().get<double>();
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
