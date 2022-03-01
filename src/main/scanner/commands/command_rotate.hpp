#ifndef COMMAND_ROTATE_H_
#define COMMAND_ROTATE_H_

#include <scanner.hpp>
#include <commands/command.hpp>

namespace scanner {
    class command_rotate : public command {
        private:
            int direction,angle;
        public:
            command_rotate(scanner* ctx, int code, int direct);
            void execute() override;
    };
}

#endif