#ifndef COMMAND_IOSTART_H_
#define COMMAND_IOSTART_H_

#include <scanner.hpp>
#include <command/command.hpp>

namespace scanner {
    class command_iostart : public command {
        public:
            command_iostart(scanner ctx, int code);
            command_iostart(scanner ctx, jcommand jcomm);
            void execute() override;
    };
}

#endif