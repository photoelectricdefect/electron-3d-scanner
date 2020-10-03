#include <models/err.hpp>

namespace scanner {
    void to_json(nlohmann::json& j, const jerror& data) {
        j = nlohmann::json{{ "msg", data.msg }};
    }

    void from_json(const nlohmann::json& j, jerror& data) {
        j.at("msg").get_to(data.msg);
    }

            err::err(std::string msg) {
                _msg = msg;
            };

std::string err::msg() {
    return _msg;
}

std::string err::json() {
    jerror err = {_msg};
    nlohmann::json j = err;
    return j.dump();
}
}
