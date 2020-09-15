#include <command.hpp>

namespace scanner {
    void command::to_json(nlohmann::json& j, const jcommand& data) {
        j = nlohmann::json{{ "code", data.code }};
    }

    void command::from_json(const nlohmann::json& j, jcommand& data) {
        j.at("code").get_to(data.code);
    }

    command::command(int _code) {
        code = _code;
    }

    command::command(jcommand jcomm) {
        code = jcomm.code;
    }
}