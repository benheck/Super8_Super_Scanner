#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

bool negative = true; // Flag to indicate if the image is negative
int sprocketSlider = 20;
int sprocketThreshold = 220; // Threshold for sprocket hole detection (200 + this max 255)

int redAdjustment = 30; // Adjustment for red channel
int greenAdjustment = 80;
int blueAdjustment = 20; // Adjustment for blue channel

enum states {
    setup,
    STATE_SCANNING,
    STATE_DONE,
    STATE_EXIT
};

states currentState = setup; // Initialize the current state

cv::Mat image, gray, binary;
lccv::PiCamera cam;

void on_trackbar(int, void*) {
    sprocketThreshold = sprocketSlider + 200;

    if (sprocketThreshold > 255) {
        sprocketThreshold = 255; // Cap the threshold value at 255
    }
    if (sprocketThreshold < 200) {
        sprocketThreshold = 200; // Cap the threshold value at 0
    }

}

void on_negative(int state, void*)
{
    negative = (state == 1); // Update the negative flag based on the checkbox state
}

void on_button_click(int state, void*) {
    std::cout << "Button clicked! State: " << state << std::endl;
}

void monitor_loop() {
    
    if (!cam.getVideoFrame(image, 1000)) { // Capture a video frame from the camera
        return;
    }

    cv::imshow("SCANNER", image);

    if (cv::waitKey(20) == 27) { // Exit on 'ESC' key at 50 HZ
        currentState = STATE_EXIT; // Set state to exit
    }

    return;

    //std::cout << "Captured frame size: " << image.cols << "x" << image.rows << std::endl;
    //std::cout << "Captured frame type: " << image.type() << std::endl;

    // Convert the image to grayscale
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Apply binary thresholding to isolate the black sprocket hole
    cv::threshold(gray, binary, sprocketThreshold, 255, cv::THRESH_BINARY);

    // Find contours in the binary image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw all contours in blue
    //cv::drawContours(image, contours, -1, cv::Scalar(255, 0, 0), 2);

    // Iterate through contours to find the sprocket hole on the left side
    bool holeFound = false;
    cv::Rect sprocketBoundingBox;

    for (const auto& contour : contours) {
        cv::Rect boundingBox = cv::boundingRect(contour);

        // Check if the bounding box is on the left side of the frame
        if (boundingBox.x < image.cols / 4 && boundingBox.width > 50 && boundingBox.height > 70) {
            // Draw a rectangle around the detected sprocket hole
            cv::rectangle(image, boundingBox, cv::Scalar(0, 255, 0), 2);
            sprocketBoundingBox = boundingBox;
            holeFound = true;
            break; // Stop after detecting the first sprocket hole
        }
    }

    // Debug: Print a message if no hole is found
    // if (!holeFound) {
    //     std::cout << "No sprocket hole detected in frame " << i + 1 << std::endl;
    // }

    if (negative) {
        cv::bitwise_not(image, image);

        //REMOVE FILM TINT HERE
        std::vector<cv::Mat> channels;
        cv::split(image, channels); // Split the image into B, G, R channels
    
        // Subtract a constant value from the green channel
        channels[0] -= redAdjustment;
        channels[1] -= greenAdjustment; // Use the slider value to adjust green intensity
        channels[2] -= blueAdjustment;

        // Merge the channels back into the image
        cv::merge(channels, image);
    }

    // Display the processed frame
    cv::flip(image, image, 0); // Flip the image vertically

    cv::imshow("SCANNER", image);

    if (cv::waitKey(20) == 27) { // Exit on 'ESC' key at 50 HZ
        currentState = STATE_EXIT; // Set state to exit
    }

}

void setup() {

    cam.options->video_width = 1280; // Set video width to 1280 pixels
    cam.options->video_height = 720; // Set video height to 720 pixels
    cam.options->framerate = 30; // Set frame rate to 30 FPS
    cam.options->timeout = 1000; // Set timeout to 1000 ms (1 second)
    //cam.options->exposure_index = lccv::Exposure_Modes::EXPOSURE_NORMAL; // Set exposure mode to normal
    //cam.options->metering_index = lccv::Metering_Modes::METERING_CENTRE; // Set metering mode to centre
    //cam.options->awb_index = lccv::WhiteBalance_Modes::WB_AUTO; // Set white balance mode to auto
    cam.options->shutter = 0.0f; // Set shutter speed to 0 (auto)
    cam.options->gain = 0.0f; // Set gain to 0 (auto)
    cam.options->ev = 0.0f; // Set exposure compensation to 0
    cam.options->brightness = 0.0f; // Set brightness to 0 (no adjustment)
    cam.options->contrast = 0.0f; // Set contrast to 0 (no adjustment)
    cam.options->saturation = 0.0f; // Set saturation to 0 (no adjustment)
    cam.options->sharpness = 0.0f; // Set sharpness to 0 (no adjustment)
    cam.options->roi_x = 0.0f; // Set ROI x-coordinate to 0 (no ROI)
    cam.options->roi_y = 0.0f; // Set ROI y-coordinate to 0 (no ROI)
    cam.options->roi_width = 0.0f; // Set ROI width to 0 (no ROI)
    cam.options->roi_height = 0.0f; // Set ROI height to 0 (no ROI)
    //cam.options->awb_gain_r = 0.0f; // Set AWB gain red to 0 (no adjustment)
    //cam.options->awb_gain_b = 0.0f; // Set AWB gain blue to 0 (no adjustment)
    cam.options->denoise = "off"; // Set denoise mode to auto

    //cam.options->setWhiteBalance(WhiteBalance_Modes::WB_DAYLIGHT); // Lock to daylight white balance
    cam.options->setWhiteBalance(WhiteBalance_Modes::WB_TUNGSTEN); // Lock to daylight white balance
    //cam.options->setWhiteBalance(WhiteBalance_Modes::WB_INCANDESCENT); // Lock to daylight white balance

    //cam.options->awb_gain_r = 1.5f; // Set red gain
    //cam.options->awb_gain_b = 1.2f; // Set blue gain
    //cam.options->setWhiteBalance(WhiteBalance_Modes::WB_CUSTOM); // Use custom white balance

    std::cout << "Camera configuration: "
          << "Width=" << cam.options->video_width
          << ", Height=" << cam.options->video_height
          << ", Framerate=" << cam.options->framerate
          << ", Timeout=" << cam.options->timeout << std::endl;

    if (!cam.startVideo()) {
        std::cerr << "Error: Failed to start video!" << std::endl;
    }
    else {
        std::cout << "Camera started successfully" << std::endl;
    }

    cv::namedWindow("SCANNER", cv::WINDOW_FULLSCREEN);
    //cv::createButton("My Button", on_button_click, nullptr, cv::QT_PUSH_BUTTON, false);
    //cv::createButton("Negative", on_negative, nullptr, cv::QT_CHECKBOX, false);
    cv::createTrackbar("Sprocket Threshold", "SCANNER", &sprocketSlider, 55, on_trackbar);
    cv::createTrackbar("Red Adjustment", "SCANNER", &redAdjustment, 100); // Range: 0-100
    cv::createTrackbar("Green Adjustment", "SCANNER", &greenAdjustment, 100); // Range: 0-100
    cv::createTrackbar("Blue Adjustment", "SCANNER", &blueAdjustment, 100); // Range: 0-100


}

int main() {

    setup(); // Call the setup function
    bool active = true;

    while(active) {
        switch(currentState) {
            case setup:
                monitor_loop(); // Call idle state function
                break;
            case STATE_SCANNING:
    
                break;
            case STATE_DONE:
    
                break;
            case STATE_EXIT:
                active = false;
                std::cout << "State: EXITING PROGRAM" << std::endl;
                break;
        }
    }

    return 0;

}

