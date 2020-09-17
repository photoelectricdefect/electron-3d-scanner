#ifndef COMMAND_VIDEOSTOP_H_
#define COMMAND_VIDEOSTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_videostop : public command {
        public:
            command_videostop(scanner& ctx, int code);
            command_videostop(scanner& ctx, jcommand jcomm);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif