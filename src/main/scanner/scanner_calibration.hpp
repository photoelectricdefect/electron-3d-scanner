#ifndef SCANNERCALIBRATION_H_
#define SCANNERCALIBRATION_H_

#include <opencv2/core.hpp>
#include <json.hpp>
#include <Eigen/Dense>

namespace scanner
{
    // struct jscannercalib
    // {
    //     std::vector<double> laser_plane;
    //     std::vector<double> rotation_axis_direction;
    //     std::vector<double> rotation_axis_source;
    //     // std::vector<double> RT_rotation_axis;
    // };

    // void to_json(nlohmann::json &j, const jscannercalib &data);
    // void from_json(const nlohmann::json &j, jscannercalib &data);

    struct jpoints
    {
        std::vector<double> direction;
        std::vector<double> source;
        std::vector<std::vector<double>> orbit_points;
        std::vector<std::vector<double>> center_points;
        int npoints;
        // std::vector<double> RT_rotation_axis;
    };

    void to_json(nlohmann::json &j, const jpoints &data);
    void from_json(const nlohmann::json &j, jpoints &data);

    class scanner_calibration
    {
    public:
        // cv::Size pattern_size = cv::Size(9, 6);
        // double square_size = 23;
        // double rotation_axis_radius = 80;
        // Eigen::Hyperplane<double, 3> laser_plane;
        // Eigen::Vector3d rotation_axis_direction;
        // Eigen::Vector3d rotation_axis_origin;
        // int n_calibration_images = 20;
        // double stepper_gear_ratio = 56. / 36;
        // int steps_per_calibration_image = 2;
        // int n_captures = 20;

        cv::Size pattern_size;
        double square_size;
        double rotation_axis_radius;
        Eigen::Hyperplane<double, 3> laser_plane;
        Eigen::Vector3d rotation_axis_direction;
        Eigen::Vector3d rotation_axis_origin;
        int n_calibration_images;
        double stepper_gear_ratio;
        int steps_per_calibration_image;
        int n_captures;

        scanner_calibration();

        void create_calibration_file(std::string fpath);
        void load(std::string fpath);
        void save(std::string fpath);
        void save_points(std::string fpath, const std::vector<double> &direction, const std::vector<double> &source, const std::vector<std::vector<double>> &orbit_points, const std::vector<std::vector<double>> &center_points, int npoints);
        void load_points(std::string fpath, std::vector<double> &direction, std::vector<double> &source, std::vector<std::vector<double>> &orbit_points, std::vector<std::vector<double>> &center_points, int &npoints);
        Eigen::Matrix4d get_axis_rigid_body_transform();
        
    };
}

#endif
