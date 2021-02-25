#include <commands/command.hpp>

namespace scanner {
    // void to_json(nlohmann::json& j, const jcommand& data) {
    //     j = nlohmann::json{{ "code", data.code }};
    // }

    // void from_json(const nlohmann::json& j, jcommand& data) {
    //     j.at("code").get_to(data.code);
    // }

    command::command(scanner& _ctx, int _code) : ctx(_ctx) {
        code = _code;
    }

    void command::execute(std::shared_ptr<command> self) {}
}