#ifndef COMMAND_SCANSTOP_H_
#define COMMAND_SCANSTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_scanstop: public command {
        public:
            command_scanstop(scanner& ctx, int code);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif