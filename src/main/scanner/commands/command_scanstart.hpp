#ifndef COMMAND_SCANSTART_H_
#define COMMAND_SCANSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_scanstart : public command {
        public:
            command_scanstart(scanner& ctx, int code);
            command_scanstart(scanner& ctx, jcommand jcomm);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif