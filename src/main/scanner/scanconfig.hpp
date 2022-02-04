#ifndef SCANCONFIG_H_
#define SCANCONFIG_H_

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <globals.hpp>

namespace scanner {
    class scanconfig {
        private:
        public:    
            double rotation_resolution; 
            std::string rotation_direction;

            scanconfig();
            
            void load();
    };
}

#endif
