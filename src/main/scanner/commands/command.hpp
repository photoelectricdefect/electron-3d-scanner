#ifndef COMMAND_H_
#define COMMAND_H_

#include <json.hpp>
#include <globals.hpp>
#include <memory>

namespace scanner {    
    class scanner;

    class command {
        public:
            scanner& ctx;
            int code;
            command(scanner& _ctx, int _code);
            virtual void execute(std::shared_ptr<command> self);
    };
}

#endif