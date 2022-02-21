#ifndef COMMAND_CAMERACALIBSTART_H_
#define COMMAND_CAMERACALIBSTART_H_

#include <scanner.hpp>
#include <commands/command.hpp>
#include <camera_calibration.hpp>

namespace scanner {
    class command_cameracalibstart : public command {
        public:
            command_cameracalibstart(scanner& ctx, int code);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif