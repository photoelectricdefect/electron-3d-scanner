#ifndef COMMAND_IOSTART_H_
#define COMMAND_IOSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>


//TODO: rename to mainthread
namespace scanner {
    class command_iostart : public command {
        public:
            command_iostart(scanner& ctx, int code);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif