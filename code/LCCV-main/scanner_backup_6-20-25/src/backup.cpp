#include <lccv.hpp>
#include <opencv2/opencv.hpp>

#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QPixmap>

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

cv::Mat image, gray, binary, baseColorMask, normalizedImage;
lccv::PiCamera cam;

QApplication *app; // Pointer to QApplication
QWidget *mainWindow; // Pointer to the main window
QLabel *videoLabel; // Pointer to the QLabel for the video feed

QImage matToQImage(const cv::Mat &mat) {
    // Convert cv::Mat to QImage
    if (mat.type() == CV_8UC1) {
        // Grayscale image
        return QImage(mat.data, mat.cols, mat.rows, mat.step[0], QImage::Format_Grayscale8).copy();
    } else if (mat.type() == CV_8UC3) {
        // Color image (BGR to RGB conversion)
        cv::Mat rgbMat;
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
        return QImage(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step[0], QImage::Format_RGB888).copy();
    } else if (mat.type() == CV_8UC4) {
        // Color image with alpha channel (BGRA to RGBA conversion)
        cv::Mat rgbaMat;
        cv::cvtColor(mat, rgbaMat, cv::COLOR_BGRA2RGBA);
        return QImage(rgbaMat.data, rgbaMat.cols, rgbaMat.rows, rgbaMat.step[0], QImage::Format_RGBA8888).copy();
    }

    // Handle other formats if needed
    std::cerr << "Unsupported image format: " << mat.type() << std::endl;
    return QImage(); // Return an empty QImage if the format is unsupported

}

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

void switchToVideo() {

}

void switchToStill() {
    // Set resolution to the closest to 720p
    cam.options->photo_width = 1280;  // Closest width to 720p
    cam.options->photo_height = 720; // Closest height to 720p
    cam.options->verbose = true;
}

void on_button_click(int state, void*) {
    std::cout << "Button clicked! State: " << state << std::endl;
}

void monitor_loop() {

    //Make CV matt filled with blue:
    //cv::Mat dummyImage(720, 1280, CV_8UC3, cv::Scalar(255, 0, 255)); // Create a blue image
    //cv::cvtColor(dummyImage, dummyImage, cv::COLOR_BGR2RGB); // Convert to RGB format

    
    if (!cam.getVideoFrame(image, 1000)) {
        std::cerr << "Error: Failed to capture video frame!" << std::endl;
        return;
    }

    if (image.empty()) {
        std::cerr << "Error: Captured frame is empty!" << std::endl;
        return;
    }

    //image = cv::Mat::zeros(720, 1280, CV_8UC3);
    //image.setTo(cv::Scalar(0, 0, 255)); // Red color


    // std::cout << "First pixel value (BGR): "
    // << static_cast<int>(image.at<cv::Vec3b>(0, 0)[0]) << ", "
    // << static_cast<int>(image.at<cv::Vec3b>(0, 0)[1]) << ", "
    // << static_cast<int>(image.at<cv::Vec3b>(0, 0)[2]) << std::endl;

    //cv::imshow("Captured Frame", image);
    //cv::waitKey(1); // Add a small delay to allow the frame to render

    //Convert to QImage
    QImage qImage = matToQImage(image); // Convert cv::Mat to QImage

    videoLabel->setPixmap(QPixmap::fromImage(qImage)); // Display the QImage in the QLabel

}

void monitor_loop() {
    
    if (!cam.getVideoFrame(image, 1000)) {
        std::cerr << "Error: Failed to capture video frame!" << std::endl;
        return;
    }

    if (image.empty()) {
        std::cerr << "Error: Captured frame is empty!" << std::endl;
        return;
    }

    std::cout << "Captured frame size: " << image.cols << "x" << image.rows << std::endl;
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

        //REMOVE GREEN TINT HERE
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

    // Convert the processed frame to QImage
    //QImage qImage = matToQImage(image);

    // Display the QImage in the QLabel
    //videoLabel->setPixmap(QPixmap::fromImage(qImage));

    //PUT THIS IMAGE IN THE QWINDOW

    cv::imshow("SCANNER", image);

    if (cv::waitKey(20) == 27) { // Exit on 'ESC' key at 50 HZ
        currentState = STATE_EXIT; // Set state to exit
    }

}

void state_machine()
{
    switch(currentState) {
        case setup:
            monitor_loop(); // Call idle state function
            break;
        case STATE_SCANNING:

            break;
        case STATE_DONE:

            break;
        case STATE_EXIT:
            std::cout << "State: EXITING PROGRAM" << std::endl;
            break;
    }

}

void setup() {



    //return;

    //switchToStill();

    cv::namedWindow("SCANNER", cv::WINDOW_FULLSCREEN);
    //cv::createButton("My Button", on_button_click, nullptr, cv::QT_PUSH_BUTTON, false);
    //cv::createButton("Negative", on_negative, nullptr, cv::QT_CHECKBOX, false);
    cv::createTrackbar("Threshold", "SCANNER", &sprocketSlider, 55, on_trackbar);
    cv::createTrackbar("Red Adjustment", "SCANNER", &redAdjustment, 100); // Range: 0-100
    cv::createTrackbar("Green Adjustment", "SCANNER", &greenAdjustment, 100); // Range: 0-100
    cv::createTrackbar("Blue Adjustment", "SCANNER", &blueAdjustment, 100); // Range: 0-100


}

int main(int argc, char *argv[]) {

    app = new QApplication(argc, argv); // Initialize QApplication

    // Create the main window
    mainWindow = new QWidget();
    mainWindow->setWindowTitle("Super 8 Film Scanner");

    // Create a QLabel for the picture box
    videoLabel = new QLabel();
    videoLabel->setFixedSize(1280, 720); // Set fixed size for the picture box
    videoLabel->setStyleSheet("background-color: black;"); // Set background color
    videoLabel->setScaledContents(true); // Enable scaling

    // Create sample buttons
    QPushButton *startButton = new QPushButton("Start");
    QPushButton *stopButton = new QPushButton("Stop");
    QPushButton *exportButton = new QPushButton("Export");

    // Create a layout and add widgets
    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(videoLabel); // Add the picture box
    layout->addWidget(startButton); // Add the Start button
    layout->addWidget(stopButton); // Add the Stop button
    layout->addWidget(exportButton); // Add the Export button
    mainWindow->setLayout(layout);

    // Show the window
    mainWindow->show();

    setup(); // Call the setup function

    //cam.options->device = "/dev/video0";
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
        return -1; // Exit the application if the camera fails to start
    }
    std::cout << "Camera started successfully" << std::endl;

    while(1) {
        state_machine(); // Call the state machine function
    }


    QTimer *timer = new QTimer(mainWindow); // Timer will be parented to mainWindow
    QObject::connect(timer, &QTimer::timeout, []() {
        monitor_loop(); // Call your logic loop
    });
    timer->start(20); // Call monitor_loop every 30ms

    int result = app->exec(); // Start the Qt event loop

    //cam.stopVideo(); // Stop the camera when the application exits
    return result;

    //return app->exec(); // Start the Qt event loop
}

