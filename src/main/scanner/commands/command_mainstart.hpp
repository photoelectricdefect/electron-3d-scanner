#ifndef COMMAND_MAINSTART_H_
#define COMMAND_MAINSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>


//TODO: rename to mainthread
namespace scanner {
    class command_mainstart : public command {
        public:
            command_mainstart(scanner* ctx, int code);
            void execute() override;
    };
}

#endif