{
  "targets": [
    {
      "target_name": "scanner",
      "cflags!": [ "-fno-exceptions", '-std=c++11' ],
      "cflags_cc!": [ "-fno-exceptions" ],
      "sources": ["scanner/scanner.cpp", 
                  "<!@(ls -d ../source/*.cpp)", "<!@(ls -d scanner/commands/*.cpp)"],
      'include_dirs': ["<!@(node -p \"require('node-addon-api').include\")",
                      "scanner/",  "../include", "/usr/local/include/opencv4/",
                      "/usr/include/boost/", "/usr/include/eigen3/"],
      "link_settings" : {
          "libraries" : ["`pkg-config --libs opencv4`",
                        "/usr/lib/libboost_thread.so",
                        "/usr/lib/libboost_chrono.so"]
      },
      'defines': [ 'NAPI_DISABLE_CPP_EXCEPTIONS' ],
      "cflags": [ "-std=c++11" ]
    }
  ]
}