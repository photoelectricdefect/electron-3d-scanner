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
      "sources": ["scanner/scanner.cpp", 
                  "<!@(ls -d ../source/*.cpp)", "<!@(ls -d scanner/commands/*.cpp)",
                  "<!@(ls -d scanner/helpers/*.cpp)"],
      'include_dirs': ["<!@(node -p \"require('node-addon-api').include\")",
                      "scanner/",  "../include", "/usr/local/include/opencv4/",
                      "/usr/include/boost/", "/usr/include/eigen3/"],
      "link_settings" : {
          "libraries" : ["`pkg-config --libs opencv4`",
                        "/usr/lib/libboost_thread.so",
                        "/usr/lib/libboost_chrono.so"]
      },
      "cflags": [ "-std=c++11" ]
    }
  ]
}