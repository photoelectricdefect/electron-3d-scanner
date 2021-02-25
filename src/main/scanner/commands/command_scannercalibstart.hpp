#ifndef COMMAND_SCANNERCALIBSTART_H_
#define COMMAND_SCANNERCALIBSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_scannercalibstart : public command {
        public:
            command_scannercalibstart(scanner& ctx, int code);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif