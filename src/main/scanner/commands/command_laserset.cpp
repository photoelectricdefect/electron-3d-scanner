#include <commands/command_laserset.hpp>
#include <commands/command_lambda.hpp>
#include <microcontroller.hpp>

namespace scanner {
command_laserset::command_laserset(scanner& ctx, int code, int state_)
    : command(ctx, code)
    , state(state_)
{
}

void command_laserset::execute(std::shared_ptr<command> self)
{
    auto comm = [ self, st = state ]()
    {
        bool err = false;
        nlohmann::json response;
        std::string msg="laser;" + microcontroller::format("state", st);
        self->ctx.controller.send_message(msg,response,500,err);

        if (!err)
            self->ctx.stremit(EV_LASERSET, "", true);
        else
            self->ctx.stremit(EV_ERROR, "", true);
    };

    ctx.controller.invoke_controller(std::shared_ptr<command>(new command_lambda<decltype(comm)>(self->ctx, -1, comm)));
}
}