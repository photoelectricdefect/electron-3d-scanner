#ifndef COMMAND_SCANNERCALIBSTART_H_
#define COMMAND_SCANNERCALIBSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_scannercalibstart : public command {
        public:
            command_scannercalibstart(scanner& ctx, int code);
            command_scannercalibstart(scanner& ctx, jcommand jcomm);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif