#ifndef COMMAND_SCANNERCALIBSTOP_H_
#define COMMAND_SCANNERCALIBSTOP_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_scannercalibstop : public command {
        public:
            command_scannercalibstop(scanner& ctx, int code);
            command_scannercalibstop(scanner& ctx, jcommand jcomm);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif