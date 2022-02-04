#include <camera.hpp>

#ifdef __linux__ 
    #include <sys/ioctl.h>
    #include <stdio.h>
    #include <dirent.h>
    #include <fcntl.h>
#elif _WIN32
    // TODO
#endif

#include <iostream>

namespace scanner {
    camera::camera() {
        calibrated=calib.load("cameracalib.json");
    }

void camera::clear_key_camera() {
    boost::unique_lock<boost::mutex> lock(mtx_camera_keyq);
    std::queue<int> empty;
    std::swap(camera_keyq,empty);
}

void camera::set_key_camera(int keycode) {
    boost::unique_lock<boost::mutex> lock(mtx_camera_keyq);

    if(camera_keyq.size()>0) {
        std::queue<int> empty;
        std::swap(camera_keyq,empty);
    }

    camera_keyq.push(keycode);
}

int camera::get_key_camera() {
    boost::unique_lock<boost::mutex> lock(mtx_camera_keyq);

    if(camera_keyq.size()<=0) return -1;

    int keycode = camera_keyq.front();
    camera_keyq.pop();

    return keycode;
}

void camera::clear_messageq_camera() {
    boost::unique_lock<boost::mutex> lock(mtx_camera_messageq);

    while(camera_messageq.size()>0) {camera_messageq.pop(); }
}

    void camera::post_message_camera(nlohmann::json msg) {
        boost::unique_lock<boost::mutex> lock(mtx_camera_messageq);
        camera_messageq.push(msg);
    }
    
    bool camera::recieve_message_camera(nlohmann::json& msg) {
        boost::unique_lock<boost::mutex> lock(mtx_camera_messageq);
        
        if(camera_messageq.size()>0) {
            msg=camera_messageq.front();
            camera_messageq.pop();

            return true;
        }
        
        return false;
    }

    #ifdef __linux__ 
        std::vector<std::pair<int,v4l2_capability>> camera::get_v4l2_capabilities() {
            std::vector<std::pair<int,v4l2_capability>> vcaps;
            DIR* dp=opendir("/dev");
            struct dirent* entry;

            while((entry=readdir(dp))!=NULL) {
                char* pos;

                if((pos=strstr(entry->d_name,"video"))!=NULL) {
                    char fpath[1024];
                    memset(fpath,'\0',1024);
                    strcpy(fpath,"/dev/");
                    strcpy(fpath+5,entry->d_name);
                    int fd=open(fpath,O_RDONLY);
                    struct v4l2_capability  video_cap;
                    
                    if(ioctl(fd, VIDIOC_QUERYCAP, &video_cap)!=-1) {
                        char strid[256];
                        memset(strid,'\0',256);
                        strcpy(strid,entry->d_name+5);
                        int id;
                        sscanf(strid,"%d",&id);
                        vcaps.push_back(std::make_pair(id,video_cap));
                    }                
                }
            }
        
            return vcaps;
        }    

    //USB 2.0 Camera: USB Camera
    int camera::get_videoid(std::string camera_name) {
        auto capabilities=get_v4l2_capabilities();

        for(int i=0;i<capabilities.size();i++) {
            int videoid=capabilities[i].first;
            auto capability=capabilities[i].second;
            std::string card((const char*)capability.card);

            if(strcmp(card.c_str(),camera_name.c_str())==0&&(capability.device_caps & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE) {
                return videoid;
            }
        }

        return -1;
    }

    #elif _WIN32
    // TODO
    #endif

    std::vector<camera::camera_info> camera::get_camera_list() {
        auto capabilities=get_v4l2_capabilities();
        std::vector<camera::camera_info> cam_list;

        for(int i=0;i<capabilities.size();i++) {
            int videoid=capabilities[i].first;
            auto capability=capabilities[i].second;
            std::string card((const char*)capability.card);

            if(capability.device_caps & V4L2_CAP_VIDEO_CAPTURE == V4L2_CAP_VIDEO_CAPTURE) {
                camera::camera_info cam_info{videoid,card};
                cam_list.push_back(cam_info);
            }
        }

        return cam_list;
    }
}