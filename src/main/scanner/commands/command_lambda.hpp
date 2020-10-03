#ifndef COMMAND_LAMBDA_H_
#define COMMAND_LAMBDA_H_

#include <scanner.hpp>
#include <commands/command.hpp>
#include <memory>
#include <string>

namespace scanner {
    template<typename F>
    class command_lambda : public command {
        public:
            F fn;
            command_lambda(scanner& ctx, int code, F& _fn) : command(ctx, code), fn(_fn) {}
            command_lambda(scanner& ctx, jcommand jcomm, F& _fn) : command(ctx, jcomm), fn(_fn) {}
            void execute(std::shared_ptr<command> self) override {
                fn();
            }
    };
}

#endif