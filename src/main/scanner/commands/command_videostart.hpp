#ifndef COMMAND_VIDEOSTART_H_
#define COMMAND_VIDEOSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_videostart : public command {
        public:
            command_videostart(scanner& ctx, int code);
            command_videostart(scanner& ctx, jcommand jcomm);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif