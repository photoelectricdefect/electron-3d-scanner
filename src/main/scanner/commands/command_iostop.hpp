#ifndef COMMAND_IOSTOP_H_
#define COMMAND_IOSTOP_H_

#include <scanner.hpp>
#include <command/command.hpp>

namespace scanner {
    class command_iostop : public command {
        public:
            command_iostop(scanner ctx, int code);
            command_iostop(scanner ctx, jcommand jcomm);
            void execute() override;
    };
}

#endif