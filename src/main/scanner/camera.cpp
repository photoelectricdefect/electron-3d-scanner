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
    message_thread_camera=nullptr;
}

bool camera::get_flag_display_video() {
    boost::unique_lock<boost::mutex> lock(mutex_display_video);
    return display_video;
}

void camera::set_flag_display_video(bool value) {
    //unique:lock is scoped, DO NOT remove parentheses
    {
        boost::unique_lock<boost::mutex> lock(mutex_display_video);
        display_video=value;
    }

    condition_display_video.notify_one();
}

bool camera::get_flag_thread_video_alive() {
    boost::unique_lock<boost::mutex> lock(mutex_thread_video_alive);
    return thread_video_alive;    
}

void camera::set_flag_thread_video_alive(bool value) {
    boost::unique_lock<boost::mutex> lock(mutex_thread_video_alive);
    thread_video_alive=value;    
}

bool camera::get_flag_thread_camera_alive() {
    boost::unique_lock<boost::mutex> lock(mutex_thread_camera_alive);
    return thread_camera_alive;    
}

void camera::set_flag_thread_camera_alive(bool value) {
    boost::unique_lock<boost::mutex> lock(mutex_thread_camera_alive);
    thread_camera_alive=value;    
}

bool camera::get_flag_calibrating_camera() {
    boost::unique_lock<boost::mutex> lock(mutex_calibrating_camera);
    return calibrating_camera;    
}

void camera::set_flag_calibrating_camera(bool value) {
    boost::unique_lock<boost::mutex> lock(mutex_calibrating_camera);
    calibrating_camera=value;    
}

bool camera::get_flag_camera_calibrated() {
    boost::unique_lock<boost::mutex> lock(mutex_camera_calibrated);
    return camera_calibrated;    
}

void camera::set_flag_camera_calibrated(bool value) {
    boost::unique_lock<boost::mutex> lock(mutex_camera_calibrated);
    camera_calibrated=value;        
}

void camera::clear_message_thread_camera() {
    boost::unique_lock<boost::mutex> lock(mutex_message_thread_camera);
    delete message_thread_camera;
    message_thread_camera=nullptr;
}

void camera::set_message_thread_camera(const nlohmann::json& message) {
    boost::unique_lock<boost::mutex> lock(mutex_message_thread_camera);

    if(message_thread_camera!=nullptr)
    {
        delete message_thread_camera;
        message_thread_camera=nullptr;
    }

    message_thread_camera = new nlohmann::json;
    *message_thread_camera=message;
    condition_message_thread_camera.notify_one();
}

void camera::get_message_thread_camera(nlohmann::json& message) {
    boost::unique_lock<boost::mutex> lock(mutex_message_thread_camera);
                    
    while(message_thread_camera==nullptr)
    {
        condition_message_thread_camera.wait(lock);
    }

    message=*message_thread_camera;
    delete message_thread_camera;
    message_thread_camera=nullptr;
}

void camera::try_get_message_thread_camera(nlohmann::json* message) {
    boost::unique_lock<boost::mutex> lock(mutex_message_thread_camera);
    message=message_thread_camera;

    if(message_thread_camera!=nullptr) {
        delete message_thread_camera;
    }

    message_thread_camera=nullptr;
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