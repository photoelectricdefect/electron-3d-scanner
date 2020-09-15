#include <string>
#include <json.hpp>

namespace scanner {
    const int
        COMM_IOSTART = 0,
        COMM_IOSTOP = 1,
        COMM_VIDEOSTART = 2,
        COMM_VIDEOSTOP = 3,
        COMM_SCAN = 4,
        COMM_LOADMODEL = 5,
        COMM_SETPROP = 6;
    
    struct jcommand {
        int code;
    };

    void to_json(nlohmann::json& j, const jcommand& data);
    void from_json(const nlohmann::json& j, jcommand& data);

    class command {
        public:
            int code;     
            command(int _code);
            command(jcommand jcomm);
    };
}