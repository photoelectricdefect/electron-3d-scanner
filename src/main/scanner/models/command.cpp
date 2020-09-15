#include <models/command.hpp>
    
namespace scanner {
    void to_json(nlohmann::json& j, const jcommand& data) {
        j = nlohmann::json{{ "code", data.code }};
    }

    void from_json(const nlohmann::json& j, jcommand& data) {
        j.at("code").get_to(data.code);
    }

    command::command(int _code) {
        code = _code;
    }

    command::command(jcommand jcomm) {
        code = jcomm.code;
    }
}