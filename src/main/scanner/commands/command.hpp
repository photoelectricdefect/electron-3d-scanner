#ifndef COMMAND_H_
#define COMMAND_H_

#include <json.hpp>
#include <flags.hpp>
#include <memory>

namespace scanner {    
    struct jcommand {
        int code;
    };
    
    void to_json(nlohmann::json& j, const jcommand& data);
    void from_json(const nlohmann::json& j, jcommand& data);

    class scanner;

    class command {
        public:
            scanner& ctx;
            int code;
            command(scanner& _ctx, int _code);
            command(scanner& _ctx, jcommand jcomm);
            virtual void execute(std::shared_ptr<command> self);
    };
}

#endif