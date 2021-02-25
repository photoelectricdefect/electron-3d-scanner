#ifndef COMMAND_MICROCONTROLLERSTART_H_
#define COMMAND_MICROCONTROLLERSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_microcontrollerstart : public command {
        public:
            command_microcontrollerstart(scanner& ctx, int code);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif