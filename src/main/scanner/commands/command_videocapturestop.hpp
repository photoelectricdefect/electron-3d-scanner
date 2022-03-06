#ifndef COMMAND_VIDEOCAPTURESTOP_H_
#define COMMAND_VIDEOCAPTURESTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_videocapturestop : public command {
        public:
            command_videocapturestop(scanner* ctx, int code);
            void execute() override;
    };
}

#endif