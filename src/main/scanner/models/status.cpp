#include <status.hpp>

namespace scanner {
    void status::to_json(nlohmann::json& j, const jstatus& data) {
        j = nlohmann::json{{ "desc", data.desc }, { "code", data.code }};
    }

    void status::from_json(const nlohmann::json& j, jstatus& data) {
        j.at("desc").get_to(data.desc);
        j.at("code").get_to(data.code);
    }

    status::status(int _code, std::string desc) {
        code = _code;
        _desc = desc;
    }

    std::string status::desc() {
        return _desc;
    }

    std::string status::json() {
        jstatus s = {_desc, code};
        nlohmann::json j = s;
        return j.dump();
    }
}