#ifndef COMMAND_GETPROP_H_
#define COMMAND_GETPROP_H_

#include <scanner.hpp>
#include <commands/command.hpp>
#include <memory>
#include <string>

namespace scanner {
    template<typename F>
    class command_setprop : public command {
        public:
            F& fn;
            command_setprop(F& _fn, scanner& ctx, int code) : command(ctx, code), json(_json) {}
            void execute(std::shared_ptr<command> self) override {
                fn();
            }
    };
}

#endif