#ifndef COMMAND_CAMERACALIBSTOP_H_
#define COMMAND_CAMERACALIBSTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_cameracalibstop : public command {
        public:
            command_cameracalibstop(scanner* ctx, int code);
            void execute() override;
    };
}

#endif