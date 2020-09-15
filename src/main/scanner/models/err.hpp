#include <json.hpp>

namespace scanner {
    struct jerror {
        std::string msg;
    };
    
    class err {
        private:
            std::string _msg;

            void to_json(nlohmann::json& j, const jerror& data);
            void from_json(const nlohmann::json& j, jerror& data);

        public:
            err(std::string msg);
            std::string msg();
            std::string json();
    };
}