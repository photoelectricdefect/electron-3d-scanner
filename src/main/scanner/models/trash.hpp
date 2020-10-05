// #include <json.hpp>

// namespace scanner {
//     const int STATUS_OK = 0,
//             STATUS_ERROR = 1;

//     struct jstatus {
//         std::string desc;
//         int code;
//     };

//     void to_json(nlohmann::json& j, const jstatus& data);
//     void from_json(const nlohmann::json& j, jstatus& data);

//     class status {
//         private:
//             std::string desc_;
//         public:
//             int code;

//             status(int code_, std::string desc);
//             std::string desc();
//             std::string json();
//     };
// }
