#include <commands/command_rotate.hpp>
#include <commands/command_lambda.hpp>
#include <microcontroller.hpp>

namespace scanner {
    command_rotate::command_rotate(scanner* ctx, int code, int direct) : command(ctx, code), direction(direct) {}

        void command_rotate::execute() {
            auto comm = [ctx=ctx,direct=direction]() {

            const int DELAY_ROTATION=1000;

            bool err=false;
			nlohmann::json response;                        
            // std::string msg="rotate;"+microcontroller::format("angle", 7.2f)+microcontroller::format("direction", direct);
            // ctx->controller.send_message(msg,response,500,err);
            
            ctx->controller.rotate(ctx->scconfig.rotation_direction, 1,DELAY_ROTATION, response, DELAY_ROTATION+1000, err);

                if(!err) ctx->stremit(EV_ROTATE,"",true);
                else ctx->stremit(EV_ERROR,"",true);
            };
            
            ctx->controller.invoke_controller(std::shared_ptr<command>(new command_lambda<decltype(comm)>(ctx,-1,comm)));
        }
}