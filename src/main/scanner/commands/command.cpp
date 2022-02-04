#include <commands/command.hpp>

namespace scanner {
    command::command(scanner& _ctx, int _code) : ctx(_ctx) {
        code = _code;
    }

    void command::execute(std::shared_ptr<command> self) {}
}