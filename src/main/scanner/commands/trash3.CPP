#include <commands/command_scannerstart.hpp>
#include <json.hpp>


namespace scanner {
    command_scannerstart::command_scannerstart(scanner& ctx, int code) : command(ctx, code) {};
    command_scannerstart::command_scannerstart(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {};

    void command_scannerstart::execute(std::shared_ptr<command> self) {
      ctx.scanner_alive = true;

		auto fn = [self]() {
			bool running = true;
		
			while(running) {
				try {
					nlohmann::json input = self->ctx.microcontroller.inputq.dequeue();
					//TODO: write to serial
					nlohmann::json output; //TODO: timed read
					self->ctx.microcontroller.outputq.enqueue(output);
				}
				catch(boost::thread_interrupted&) { running = false; }
			}
		};

      nlohmann::json j;
      j["prop"] = PROP_SCANNERALIVE;
      j["value"] = true;
      ctx.stremit(EV_PROPCHANGED, j.dump(), true); 
      ctx.microcontroller.thread_microcontroller = boost::thread{fn};
    }
}