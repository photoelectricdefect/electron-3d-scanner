#ifndef COMMAND_EMITEVENT_H_
#define COMMAND_EMITEVENT_H_

#include <scanner.hpp>
#include <commands/command.hpp>
#include <models/event.hpp>
#include <memory>
#include <string>
#include <flags.hpp>

namespace scanner {
    template<typename T>
    class command_input : public command {
        public:
            event<T> input;
            std::string channel;

            command_input(scanner& ctx, int code, event<T> _input, std::string _channel) : command(ctx, code), input(_input), channel(_channel) {}
            command_input(scanner& ctx, jcommand jcomm, event<T> _input, std::string _channel) : command(ctx, jcomm), input(_input), channel(_channel) {}

            void execute(std::shared_ptr<command> self) override {
                if(channel == EVCHANNEL_CAMERA) self->ctx.camera_inputq.enqueue(input);
                if(channel == EVCHANNEL_TABLE) self->ctx.table_inputq.enqueue(input);
            }
    };
}

#endif