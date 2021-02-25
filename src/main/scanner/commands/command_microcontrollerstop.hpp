#ifndef COMMAND_MICROCONTROLLERSTOP_H_
#define COMMAND_MICROCONTROLLERSTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_microcontrollerstop : public command {
        public:
            command_microcontrollerstop(scanner& ctx, int code);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif