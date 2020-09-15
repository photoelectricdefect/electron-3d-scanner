#include <json.hpp>

namespace scanner {
    struct jerror {
        std::string msg;
    };

    void to_json(nlohmann::json& j, const jerror& data);
    void from_json(const nlohmann::json& j, jerror& data);
    
    class err {
        private:
            std::string _msg;
        public:
            err(std::string msg);
            std::string msg();
            std::string json();
    };
}