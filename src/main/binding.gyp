{
  "targets": [
    {
      "target_name": "scanner",
  'cflags!': [ '-fno-exceptions' ],
  'cflags_cc!': [ '-fno-exceptions' ],
  'xcode_settings': {
    'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
    'CLANG_CXX_LIBRARY': 'libc++',
    'MACOSX_DEPLOYMENT_TARGET': '10.7',
  },
  'msvs_settings': {
    'VCCLCompilerTool': { 'ExceptionHandling': 1 },
  },
      "sources": ["<!@(find ./scanner -name '*.cpp')",
                  "<!@(find ../external/serial/src -name '*.cc')",
                  "../external/base64/base64.cpp"],
      'include_dirs': ["<!@(node -p \"require('node-addon-api').include\")",
                      "<!@(find scanner/ -type d -not -path '*/\.*')",  
                      "../external/base64", 
                      "../external/nlohmann",
                      "../external/serial/include", 
                      "/usr/include/opencv4/",
                      "/usr/include/boost/", 
                      "/usr/include/eigen3/"],
      "link_settings" : {
          "libraries" : ["`pkg-config --libs opencv4`",
                        "/usr/lib/libboost_thread.so",
                        "/usr/lib/libboost_chrono.so",
                        "/usr/lib/libboost_system.so"]
      },
      "cflags": [ "-std=c++17" ]
      # "ldflags": [
      #     "-Wl,-z,defs"
      # ]
    }
  ]
}