#ifndef LCCV_HPP
#define LCCV_HPP

#include <mutex>
#include <atomic>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "libcamera_app.hpp"
#include "libcamera_app_options.hpp"

namespace lccv {

    class PiCamera {

        public:
            PiCamera();
            PiCamera(uint32_t id);
            ~PiCamera();

            Options *options;

            //Photo mode
            bool startPhoto();
            bool capturePhoto(cv::Mat &frame);
            bool stopPhoto();

            //Video mode
            bool startVideo();
            bool getVideoFrame(cv::Mat &frame, unsigned int timeout);
            void stopVideo();

            //Applies new zoom options. Before invoking this func modify options->roi.
            void ApplyZoomOptions();

            void setControls(const libcamera::ControlList &controls);

        protected:
            void run();

            std::unique_ptr<LibcameraApp> app;
            void getImage(cv::Mat &frame, CompletedRequestPtr &payload);
            static void *videoThreadFunc(void *p);
            pthread_t videothread;
            unsigned int still_flags;
            unsigned int vw,vh,vstr;
            std::atomic<bool> running,frameready;
            uint8_t *framebuffer;
            std::mutex mtx;
            bool camerastarted;
            bool cacheInvalidated; // Add this to the PiCamera class

        private:
            libcamera::Camera *camera_;


    };

}
#endif
