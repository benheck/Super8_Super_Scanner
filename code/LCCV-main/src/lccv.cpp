#include "lccv.hpp"
#include <libcamera/libcamera/stream.h>
#include <time.h>
#include <iostream>

using namespace cv;
using namespace lccv;

PiCamera::PiCamera() : PiCamera(0) {}

PiCamera::PiCamera(uint32_t id) {
	app = std::make_unique<LibcameraApp>(std::make_unique<Options>());
    options = static_cast<Options *>(app->GetOptions());

    options->setApp(app.get()); // Pass the LibcameraApp instance to Options

    still_flags = LibcameraApp::FLAG_STILL_NONE;
	options->camera = id;
    options->photo_width = 4056;
    options->photo_height = 3040;
    options->video_width = 640;
    options->video_height = 480;
    options->framerate = 30;
    options->denoise = "auto";
    options->timeout = 1000;
    options->setMetering(Metering_Modes::METERING_MATRIX);
    options->setExposureMode(Exposure_Modes::EXPOSURE_NORMAL);
    options->setWhiteBalance(WhiteBalance_Modes::WB_AUTO);
    options->contrast = 1.0f;
    options->saturation = 1.0f; 
    still_flags |= LibcameraApp::FLAG_STILL_RGB;
    running.store(false, std::memory_order_release);;
    frameready.store(false, std::memory_order_release);;
    framebuffer=nullptr;
    camerastarted=false;

}

PiCamera::~PiCamera() {}

void PiCamera::setControls(const libcamera::ControlList &controls) {
    if (app) {
        app->SetControls(const_cast<libcamera::ControlList &>(controls)); // Apply the controls
    } else {
        std::cerr << "Error: LibcameraApp instance is null!" << std::endl;
    }
}

void PiCamera::getImage(cv::Mat &frame, CompletedRequestPtr &payload) {
    static libcamera::Stream *cachedStream = nullptr;
    static unsigned int cachedWidth = 0, cachedHeight = 0, cachedStride = 0;

    // Reset the cache if invalidated
    if (cacheInvalidated) {
        std::cout << "Resetting cached stream and dimensions..." << std::endl;
        cachedStream = nullptr;
        cachedWidth = 0;
        cachedHeight = 0;
        cachedStride = 0;
        cacheInvalidated = false; // Mark the cache as valid again
    }

    // Cache the stream and its dimensions on the first call
    if (!cachedStream) {
        std::cout << "Caching stream and dimensions..." << std::endl;
        cachedStream = app->StillStream();
        app->StreamDimensions(cachedStream, &cachedWidth, &cachedHeight, &cachedStride);
    }

    // Print debug info
    //std::cout << "----------------------Width: " << cachedWidth << ", Height: " << cachedHeight << ", Stride: " << cachedStride << std::endl;

    // Map the memory for the current payload
    const std::vector<libcamera::Span<uint8_t>> mem = app->Mmap(payload->buffers[cachedStream]);

    // Create the OpenCV matrix
    frame.create(cachedHeight, cachedWidth, CV_8UC3);

    // Copy the data from the camera buffer to the OpenCV matrix
    uint ls = cachedWidth * 3; // Line size in bytes
    uint8_t *ptr = (uint8_t *)mem[0].data();

    for (unsigned int i = 0; i < cachedHeight; i++, ptr += cachedStride) {
        memcpy(frame.ptr(i), ptr, ls);
    }
}

bool PiCamera::startPhoto() {
    app->OpenCamera();
    app->ConfigureStill(still_flags);
    app->StartCamera();
    camerastarted = true;
    cacheInvalidated = true; // Invalidate the cache when photo mode starts
    return true;
}

bool PiCamera::stopPhoto() {
    if (camerastarted) {
        camerastarted = false;
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();

        cacheInvalidated = true; // Invalidate the cache when photo mode stops
    }
    return true;
}

bool PiCamera::capturePhoto(cv::Mat &frame) {
    // Ensure the camera is in a started state
    if (!camerastarted) {
        std::cerr << "Error: Camera is not started! Call startPhoto() first." << std::endl;
        return false;
    }

    // Wait for a message from the camera
    LibcameraApp::Msg msg = app->Wait();
    if (msg.type != LibcameraApp::MsgType::RequestComplete) {
        std::cerr << "Error: Invalid message type received!" << std::endl;
        return false;
    }

    // Ensure the still stream is valid
    if (!app->StillStream()) {
        std::cerr << "Error: Still stream is null!" << std::endl;
        return false;
    }

    // Extract the image from the completed request
    try {
        getImage(frame, std::get<CompletedRequestPtr>(msg.payload));
    } catch (const std::exception &e) {
        std::cerr << "Error: Failed to extract image: " << e.what() << std::endl;
        return false;
    }

    return true; // Photo captured successfully
}

bool PiCamera::startVideo()
{
    if(camerastarted)stopPhoto();
    if(running.load(std::memory_order_relaxed)){
        std::cerr<<"Video thread already running";
        return false;
    }
    frameready.store(false, std::memory_order_release);
    app->OpenCamera();
    app->ConfigureViewfinder();
    app->StartCamera();

    int ret = pthread_create(&videothread, NULL, &videoThreadFunc, this);
    if (ret != 0) {
        std::cerr<<"Error starting video thread";
        return false;
    }
    return true;
}

void PiCamera::stopVideo()
{
    if(!running)return;

    running.store(false, std::memory_order_release);;

    //join thread
    void *status;
    int ret = pthread_join(videothread, &status);
    if(ret<0)
        std::cerr<<"Error joining thread"<<std::endl;

    app->StopCamera();
    app->Teardown();
    app->CloseCamera();
    frameready.store(false, std::memory_order_release);;
}

bool PiCamera::getVideoFrame(cv::Mat &frame, unsigned int timeout)
{
    if(!running.load(std::memory_order_acquire))return false;
    auto start_time = std::chrono::high_resolution_clock::now();
    bool timeout_reached = false;
    timespec req;
    req.tv_sec=0;
    req.tv_nsec=1000000;//1ms
    while((!frameready.load(std::memory_order_acquire))&&(!timeout_reached)){
        nanosleep(&req,NULL);
        timeout_reached = (std::chrono::high_resolution_clock::now() - start_time > std::chrono::milliseconds(timeout));
    }
    if(frameready.load(std::memory_order_acquire)){
        frame.create(vh,vw,CV_8UC3);
        uint ls = vw*3;
        mtx.lock();
            uint8_t *ptr = framebuffer;
            for (unsigned int i = 0; i < vh; i++, ptr += vstr)
                memcpy(frame.ptr(i),ptr,ls);
        mtx.unlock();
        frameready.store(false, std::memory_order_release);;
        return true;
    }
    else
        return false;
}

void *PiCamera::videoThreadFunc(void *p)
{
    PiCamera *t = (PiCamera *)p;
    t->running.store(true, std::memory_order_release);
    //allocate framebuffer
    //unsigned int vw,vh,vstr;
    libcamera::Stream *stream = t->app->ViewfinderStream(&t->vw,&t->vh,&t->vstr);
    int buffersize=t->vh*t->vstr;
    if(t->framebuffer)delete[] t->framebuffer;
    t->framebuffer=new uint8_t[buffersize];
    std::vector<libcamera::Span<uint8_t>> mem;

    //main loop
    while(t->running.load(std::memory_order_acquire)){
        LibcameraApp::Msg msg = t->app->Wait();
        if (msg.type == LibcameraApp::MsgType::Quit){
            std::cerr<<"Quit message received"<<std::endl;
            t->running.store(false,std::memory_order_release);
        }
        else if (msg.type != LibcameraApp::MsgType::RequestComplete)
            throw std::runtime_error("unrecognised message!");


        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
        mem = t->app->Mmap(payload->buffers[stream]);
        t->mtx.lock();
            memcpy(t->framebuffer,mem[0].data(),buffersize);
        t->mtx.unlock();
        t->frameready.store(true, std::memory_order_release);
    }
    if(t->framebuffer){
        delete[] t->framebuffer;
        t->framebuffer=nullptr;
    }
    return NULL;
}

void PiCamera::ApplyZoomOptions()
{
    app->ApplyRoiSettings();
}
