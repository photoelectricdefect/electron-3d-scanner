#ifndef COMMAND_VIDEOCAPTURESTART_H_
#define COMMAND_VIDEOCAPTURESTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_videocapturestart : public command {
        public:
            command_videocapturestart(scanner* ctx, int code);
            void execute() override;
    };
}

#endif