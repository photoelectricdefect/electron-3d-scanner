#ifndef COMMAND_LASERSET_H_
#define COMMAND_LASERSET_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_laserset : public command {
        private:
            int state;
        public:
            command_laserset(scanner& ctx, int code, int state_);
            void execute(std::shared_ptr<command> self) override;
    };
}

#endif