#ifndef COMMAND_IOSTOP_H_
#define COMMAND_IOSTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

//TODO: rename to mainthread
namespace scanner {
    class command_iostop : public command {
        public:
            command_iostop(scanner& ctx, int code);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif