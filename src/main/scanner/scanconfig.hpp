#ifndef SCANCONFIG_H_
#define SCANCONFIG_H_

#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace scanner {
    class scanconfig {
        public:    
            float angle, step_resolution; 
            int direction;

            scanconfig();
            
            void load();
    };
}

#endif
