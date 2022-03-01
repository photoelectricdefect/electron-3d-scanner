#include <commands/command.hpp>

namespace scanner {
    command::command(scanner* ctx_, int code_) : ctx(ctx_), code(code_) {}

    void command::execute() {}
}