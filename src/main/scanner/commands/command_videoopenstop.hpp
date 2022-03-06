#ifndef COMMAND_VIDEOOPENSTOP_H_
#define COMMAND_VIDEOOPENSTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_videoopenstop : public command {
        public:
            command_videoopenstop(scanner* ctx, int code);
            void execute() override;
    };
}

#endif