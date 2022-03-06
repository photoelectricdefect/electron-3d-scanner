#ifndef COMMAND_VIDEOOPENSTART_H_
#define COMMAND_VIDEOOPENSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_videoopenstart : public command {
        public:
            command_videoopenstart(scanner* ctx, int code);
            void execute() override;
    };
}

#endif