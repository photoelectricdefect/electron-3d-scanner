#include <models/command.hpp>
    
namespace scanner {
    void to_json(nlohmann::json& j, const jcommand& data) {
        j = nlohmann::json{{ "code", data.code }};
    }

    void from_json(const nlohmann::json& j, jcommand& data) {
        j.at("code").get_to(data.code);
    }

    command::command(scanner& ctx, int _code) {
        _ctx = ctx;
        code = _code;
    }

    command::command(scanner& ctx, jcommand jcomm) {
        _ctx = ctx;
        code = jcomm.code;
    }

    virtual void command::execute() {};
}