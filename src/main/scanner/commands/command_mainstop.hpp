#ifndef COMMAND_MAINSTOP_H_
#define COMMAND_MAINSTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

//TODO: rename to mainthread
namespace scanner {
    class command_mainstop : public command {
        public:
            command_mainstop(scanner* ctx, int code);
            void execute() override;
    };
}

#endif