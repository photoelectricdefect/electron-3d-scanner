// #include <napi.h>
// #include <map>
// #include <string>
// #include <iostream>
// #include <models/err.hpp>
// #include <models/status.hpp>
// #include <models/command.hpp>
// #include <scanner.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

// namespace scanner {

// const scanner::scanner sc;
// const std::map<std::string, Napi::FunctionReference> ev_handlers;

// void list_handlers() {

//     std::map<std::string, Napi::FunctionReference>::iterator it = ev_handlers.begin();
    
//     while(it != ev_handlers.end()) {
//         std::cout << "key: " << it->first << std::endl;
//         it++;
//     }

//     std::cout << "------------------------" << std::endl;
// }

// void add_listener(const Napi::CallbackInfo& info) {
//     Napi::Function fn = info[1].As<Napi::Function>();
//     std::string e = info[0].As<Napi::String>().Utf8Value() ;
//     std::map<std::string, Napi::FunctionReference>::iterator it = ev_handlers.find(e);
    
//     if(it == ev_handlers.end()) ev_handlers.insert(std::pair<std::string, Napi::FunctionReference>(e, Napi::Persistent(fn)));
//     else it->second = Napi::Persistent(fn);
// }

// void remove_listener(const Napi::CallbackInfo& info) {
//     std::string e = info[0].As<Napi::String>().Utf8Value() ;
//     ev_handlers.erase(e);
// }

// void send_command(const Napi::CallbackInfo& info) {
//     std::string comm = info[0].As<Napi::String>().Utf8Value();
//     nlohmann::json j = nlohmann::json::parse(comm);
//     command comm = j.get<jcommand>();
//     sc.try_send_command(comm);
// }

// void stremit(std::string e) { stremit(e, ""); }

// void stremit(std::string e, Napi::String str) {    
//     std::map<std::string, Napi::FunctionReference>::iterator it = ev_handlers.find(e);
    
//     if(it != ev_handlers.end()) {
//         Napi::Function fn = it->second.Value();        
//         fn.Call({str});
//     }
// }

// // Napi::Value on_error(const Napi::CallbackInfo& info) {
// //     Napi::String msg = info[0].As<Napi::String>();
// //     scanner::err err(msg.Utf8Value());
// //     stremit(EV_ERROR, Napi::String::New(info.Env(), err.json()));

// //     return Napi::String::New(info.Env(), "faaaaccck");
// // }

// // //TODO
// // Napi::Value on_error(error err) {
    
// //     return Napi::String::New(info.Env(), "faaaaccck");
// // }

// // //TODO
// // void on_status(status status) {
    
// // }

// Napi::Object init(Napi::Env env, Napi::Object exports) {
//     exports.Set(Napi::String::New(env, "addListener"), 
//                     Napi::Function::New(env, add_listener));

//     exports.Set(Napi::String::New(env, "removeListener"), 
//                     Napi::Function::New(env, remove_listener));

//     exports.Set(Napi::String::New(env, "sendCommand"), 
//                     Napi::Function::New(env, send_command));

//     // exports.Set(Napi::String::New(env, "test_imemit"), 
//     //                 Napi::Function::New(env, imemit));

//     return exports;
// }
// }

// NODE_API_MODULE(NODE_GYP_MODULE_NAME, scanner::init)