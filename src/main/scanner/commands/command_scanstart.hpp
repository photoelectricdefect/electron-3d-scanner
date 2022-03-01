#ifndef COMMAND_SCANSTART_H_
#define COMMAND_SCANSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_scanstart : public command {
        public:
            command_scanstart(scanner* ctx, int code);
            void execute() override;
    };
}

#endif