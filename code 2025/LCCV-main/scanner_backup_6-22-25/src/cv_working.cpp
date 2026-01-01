#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

bool negative = true; // Flag to indicate if the image is negative
int sprocketSlider = 20;
int sprocketThreshold = 220; // Threshold for sprocket hole detection (200 + this max 255)

//Bombsights for where we sample base film color. Adjust gate edges and KODAK text
//NOTE: Scale these during the 4k scan along with gaussian blur
int sampleXoffset = 25; // Offset for sampling the film color (center to left of sprocket center)
int sampleYoffset = 180; // Offset +- center of sprocket, in pixels

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
        sprocketThreshold = 200; // Cap the threshold value at 200
    }

}

void cameraSetup() {
	uint32_t num_cams = LibcameraApp::GetNumberCameras();
	std::cout << "Found " << num_cams << " cameras." << std::endl;
    std::cout<<"Sample program for LCCV video capture"<<std::endl;
    std::cout<<"Press ESC to stop."<<std::endl;

    cam.options->video_width=1280;
    cam.options->video_height=720;
    cam.options->framerate=30;

    //MAKE OPTIONS:
    cam.options->setWhiteBalance(WhiteBalance_Modes::WB_INCANDESCENT); // Lock to daylight white balance
 
    
    if (!cam.startVideo()) {
        std::cerr << "Error: Failed to start video!" << std::endl;
        exit(-1); // Exit if the camera fails to start
    }

    cv::namedWindow("SCANNER", cv::WINDOW_FULLSCREEN);
    cv::createTrackbar("Sprocket Threshold", "SCANNER", &sprocketSlider, 55, on_trackbar);
    cv::createTrackbar("Film Color Sample X Offset", "SCANNER", &sampleXoffset, 50); // Range: 0-50
    cv::createTrackbar("Film Color Sample Y Offset", "SCANNER", &sampleYoffset, 320); // Range: 0-300
    //cv::createTrackbar("Red Adjustment", "SCANNER", &redAdjustment, 100); // Range: 0-100
    //cv::createTrackbar("Green Adjustment", "SCANNER", &greenAdjustment, 100); // Range: 0-100
    //cv::createTrackbar("Blue Adjustment", "SCANNER", &blueAdjustment, 100); // Range: 0-100



}

void monitor_loop() {
    
    if (!cam.getVideoFrame(image, 1000)) { // Capture a video frame from the camera
        std::cerr << "Error: Failed to capture video frame!" << std::endl;
        return;
    }

    //std::cout << "Captured frame size: " << image.cols << "x" << image.rows << std::endl;
    //std::cout << "Captured frame type: " << image.type() << std::endl;

    // Convert the image to grayscale
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Apply binary thresholding to isolate the black sprocket hole
    cv::threshold(gray, binary, sprocketThreshold, 255, cv::THRESH_BINARY);

    // Find contours in the binary image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Iterate through contours to find the sprocket hole on the left side
    bool holeFound = false;
    cv::Rect sprocketBoundingBox;

    int centerX;
    int centerY;

    for (const auto& contour : contours) {
        cv::Rect boundingBox = cv::boundingRect(contour);

        // Check if the bounding box is on the left side of the frame
        if (boundingBox.x < image.cols / 4 && boundingBox.width > 50 && boundingBox.height > 60) {
            // Draw a rectangle around the detected sprocket hole
            cv::rectangle(image, boundingBox, cv::Scalar(0, 255, 0), 2);
            sprocketBoundingBox = boundingBox;
            holeFound = true;

            //Calc center of boundingBox:
            centerX = boundingBox.x + (boundingBox.width / 2);
            centerY = boundingBox.y + (boundingBox.height / 2);

            //Stay between sloppy gate image and the occasional "KODAK" text
            centerX -= sampleXoffset; // Offset the centerX by the sampleXoffset value

            break; // Stop after detecting the first sprocket hole
        }
    }

    if (holeFound && negative) {

        cv::bitwise_not(image, image);        //Invert image
        cv::Mat blurImage = image.clone();      //Copy
        cv::GaussianBlur(blurImage, blurImage, cv::Size(5, 5), 0);  //Blur copy

        cv::Vec3b pixelValue;

        //Get base film color from either above or below the sprocket hole depending on vertical position
        if (centerY < 360) { //Top half? Detect below hole
            pixelValue = blurImage.at<cv::Vec3b>(centerY + sampleYoffset, centerX);
            cv::circle(image, cv::Point(centerX, centerY - sampleYoffset), 20, cv::Scalar(0, 0, 128), -1);
            cv::circle(image, cv::Point(centerX, centerY + sampleYoffset), 20, cv::Scalar(0, 0, 255), -1);
        }
        else { //Bottom half? Detect above hole
            pixelValue = blurImage.at<cv::Vec3b>(centerY - sampleYoffset, centerX); 
            cv::circle(image, cv::Point(centerX, centerY - sampleYoffset), 20, cv::Scalar(0, 0, 255), -1);
            cv::circle(image, cv::Point(centerX, centerY + sampleYoffset), 20, cv::Scalar(0, 0, 128), -1);
        }

        //Get the RGB values of the area just left of the sprocket hole
        //cv::Vec3b pixelValue = processedImage.at<cv::Vec3b>(sprocketBoundingBox.y - 20, sprocketBoundingBox.x);
        int redValue = pixelValue[2]; // Red channel value
        int greenValue = pixelValue[1]; // Green channel value
        int blueValue = pixelValue[0]; // Blue channel value

        //Remove those colors from image:
        cv::subtract(image, cv::Scalar(blueValue, greenValue, redValue), image); // Subtract the pixel value from the image


    }

    cv::flip(image, image, 0);      // Flip the image vertically

    cv::imshow("SCANNER", image);

    if (cv::waitKey(20) == 27) {    // Exit on 'ESC' key at 50 HZ
        currentState = STATE_EXIT; // Set state to exit
    }

}

int main() {

    cameraSetup();
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
                break;
        }
    
    }

    cam.stopVideo();
	cv::destroyAllWindows();
}
