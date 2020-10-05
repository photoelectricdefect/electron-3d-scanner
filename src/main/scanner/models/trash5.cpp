// #include <models/config.hpp>


// //TODO: move to cameracalib
// namespace scanner {
//     void to_json(nlohmann::json& j, const jcalib& data) {
//         j = nlohmann::json{{ "K", data.K }, { "D", data.D }};
//     }

//     void from_json(const nlohmann::json& j, jcalib& data) {
//         j.at("K").get_to(data.K);
//         j.at("D").get_to(data.D);
//     }


// void config::save(const cv::Mat& K, const cv::Mat& D) {
//     std::vector<double> K_, D_;

//     for(int i = 0; i < K.rows; i++) {
//         for(int j = 0; j < K.cols; j++) {
//             K_.push_back(K.ptr<double>(i)[j]);
//         }
//     }

//     for(int i = 0; i < D.rows; i++) {
//         for(int j = 0; j < D.cols; j++) {
//             D_.push_back(D.ptr<double>(i)[j]);
//         }
//     }

//     jcalib data = {_K, _D};
//     nlohmann::json j = data;
//     std::ofstream file;
//     file.open("calib.json");
//     file << j.dump();
//     file.close();
// }

// void config::load(cv::Mat& K, cv::Mat& D) {
//     std::ifstream file("calib.json");
//     nlohmann::json j;
//     file >> j;
//     jcalib data = j.get<jcalib>();

//     //Handle no existing save data
//     //if(data.K.size() == 0 || data.D.size() == 0) return false;

//     K = cv::Mat(cv::Size(3, 3), CV_64FC1);
//     D = cv::Mat(cv::Size(1, 4), CV_64FC1);

//     for(int i = 0; i < K.rows; i++) {
//         for(int j = 0; j < K.cols; j++) {
//             K.ptr<double>(i)[j] = data.K[i * K.cols + j];
//         }
//     }

//     for(int i = 0; i < D.rows; i++) {
//         for(int j = 0; j < D.cols; j++) {
//             D.ptr<double>(i)[j] = data.D[i * D.cols + j];            
//         }
//     }

//     }
// }