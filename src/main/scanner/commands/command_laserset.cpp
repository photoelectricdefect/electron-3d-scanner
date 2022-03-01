#include <commands/command_laserset.hpp>
#include <commands/command_lambda.hpp>
#include <microcontroller.hpp>

namespace scanner {
command_laserset::command_laserset(scanner* ctx, int code, int state_)
    : command(ctx, code), state(state_) {}

void command_laserset::execute()
{
    auto comm = [ctx=ctx, state=state ]()
    {
        bool err = false;
        nlohmann::json response;
        ctx->controller.set_laser(state, 10, response, 2000, err);

        if (!err)
            ctx->stremit(EV_LASERSET, "", true);
        else
            ctx->stremit(EV_ERROR, "", true);
    };

    ctx->controller.invoke_controller(std::shared_ptr<command>(new command_lambda<decltype(comm)>(ctx, -1, comm)));
}
}