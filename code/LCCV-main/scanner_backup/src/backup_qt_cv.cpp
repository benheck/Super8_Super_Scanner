#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>

#include <QApplication>
#include <QPushButton>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <QImage>
#include <QPixmap>

#include <iostream>
#include "MarlinController.h"

bool negative = true; // Flag to indicate if the image is negative
int sprocketSlider = 20;
int sprocketThreshold = 220; // Threshold for sprocket hole detection (200 + this max 255)

//Bombsights for where we sample base film color. Adjust gate edges and KODAK text
//NOTE: Scale these during the 4k scan along with gaussian blur
int sampleXoffset = 25; // Offset for sampling the film color (center to left of sprocket center)
int sampleYoffset = 180; // Offset +- center of sprocket, in pixels

enum states {
    setup,
    STATE_FIND_NEXT_FRAME,
    STATE_SCANNING,
    STATE_DONE,
    STATE_EXIT
};

states currentState = setup; // Initialize the current state

cv::Mat image, gray, binary;
lccv::PiCamera cam; // Global camera object
QLabel *videoLabel = nullptr; // Global reference to the video label
QTimer *timer = nullptr; // Global pointer to the QTimer

MarlinController *marlin = nullptr;

// Function to convert cv::Mat to QImage
QImage matToQImage(const cv::Mat &mat) {
    if (mat.empty()) {
        return QImage();
    }
    cv::Mat rgb;
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB); // Convert BGR to RGB
    return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}

// Function to handle sprocket threshold slider changes
void onSprocketSliderChanged(int value) {
    sprocketThreshold = value + 200;
    //std::cout << "Sprocket Threshold: " << sprocketThreshold << std::endl;
}

// Function to handle X offset slider changes
void onXOffsetSliderChanged(int value) {
    sampleXoffset = value;
    //std::cout << "Film Color Sample X Offset: " << sampleXoffset << std::endl;
}

// Function to handle Y offset slider changes
void onYOffsetSliderChanged(int value) {
    sampleYoffset = value;
    //std::cout << "Film Color Sample Y Offset: " << sampleYoffset << std::endl;
}

// Button functions
void onNextButtonClicked() {
    if (marlin) {
        marlin->advanceFilm(4.0f); // Move the Y-axis forward by 10 mm
        marlin->zeroAxes();
    }
}

void onPrevButtonClicked() {

}

void onExitButtonClicked(QApplication &app) {
    std::cout << "Exit button clicked!" << std::endl;

    // Stop the QTimer
    timer->stop();
    std::cout << "QTimer stopped." << std::endl;

    // Clean up Marlin controller
    if (marlin) {
        delete marlin;
        marlin = nullptr;
        std::cout << "Marlin controller disconnected." << std::endl;
    }

    // Stop the camera
    cam.stopVideo();
    std::cout << "Camera stopped." << std::endl;

    // Quit the application
    app.quit();
}

void cameraSetup() {
    uint32_t num_cams = LibcameraApp::GetNumberCameras();
    std::cout << "Found " << num_cams << " cameras." << std::endl;
    std::cout << "Sample program for LCCV video capture" << std::endl;
    std::cout << "Press ESC to stop." << std::endl;

    cam.options->video_width = 1280;
    cam.options->video_height = 720;
    cam.options->framerate = 30;

    //MAKE OPTIONS:
    cam.options->setWhiteBalance(WhiteBalance_Modes::WB_INCANDESCENT); // Lock to daylight white balance

    if (!cam.startVideo()) {
        std::cerr << "Error: Failed to start video!" << std::endl;
        exit(-1); // Exit if the camera fails to start
    }
}

// Function to update the video feed
void updateVideoFeed() {

    if (!cam.getVideoFrame(image, 1000)) { // Capture a video frame from the camera
        std::cerr << "Error: Failed to capture video frame!" << std::endl;
        return;
    }

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

    QImage qImage = matToQImage(image); // Convert cv::Mat to QImage
    if (!qImage.isNull() && videoLabel != nullptr) {
        videoLabel->setPixmap(QPixmap::fromImage(qImage)); // Display the frame in QLabel
    }

}

void state_machine() {
    switch (currentState) {
        case STATE_SETUP:
            updateVideoFeed(); // Call update video feed in setup state
            break;
        case STATE_SCANNING:
            // Add scanning logic here
            break;
        case STATE_DONE:
            // Add done logic here
            break;
        case STATE_EXIT:
            //active = false;
            break;
    }
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    try {
        marlin = new MarlinController("/dev/ttyACM0", 115200); // Replace with your serial port and baud rate
    } catch (const std::exception &e) {
        std::cerr << "Error initializing Marlin controller: " << e.what() << std::endl;
        return -1;
    }

    cameraSetup();

    // Create the main window
    QWidget window;
    window.setWindowTitle("Video Display with Controls");
    window.resize(1800, 900); // Set the window size to 1800x900
    window.setStyleSheet("background-color: #333333;"); // Dark gray background

    // Create a video display label
    videoLabel = new QLabel(); // Assign to the global variable
    videoLabel->setFixedSize(1280, 720); // Set video display size
    videoLabel->setStyleSheet("background-color: black;"); // Black background for video

    // Create sliders
    QSlider *sprocketSliderWidget = new QSlider(Qt::Horizontal);
    sprocketSliderWidget->setRange(0, 55);
    sprocketSliderWidget->setValue(sprocketSlider);
    QObject::connect(sprocketSliderWidget, &QSlider::valueChanged, onSprocketSliderChanged);

    QSlider *xOffsetSliderWidget = new QSlider(Qt::Horizontal);
    xOffsetSliderWidget->setRange(0, 50);
    xOffsetSliderWidget->setValue(sampleXoffset);
    QObject::connect(xOffsetSliderWidget, &QSlider::valueChanged, onXOffsetSliderChanged);

    QSlider *yOffsetSliderWidget = new QSlider(Qt::Horizontal);
    yOffsetSliderWidget->setRange(140, 240);
    yOffsetSliderWidget->setValue(sampleYoffset);
    QObject::connect(yOffsetSliderWidget, &QSlider::valueChanged, onYOffsetSliderChanged);

    // Create labels for sliders
    QLabel *sprocketSliderLabel = new QLabel("Sprocket Threshold");
    sprocketSliderLabel->setStyleSheet("color: white;"); // White text for better visibility
    QLabel *xOffsetSliderLabel = new QLabel("Film Color Sample X Offset");
    xOffsetSliderLabel->setStyleSheet("color: white;");
    QLabel *yOffsetSliderLabel = new QLabel("Film Color Sample Y Offset");
    yOffsetSliderLabel->setStyleSheet("color: white;");

    // Create buttons
    QPushButton *button1 = new QPushButton("Next");
    QPushButton *button2 = new QPushButton("Prev");
    QPushButton *exitButton = new QPushButton("Exit");

    // Connect button signals
    QObject::connect(button1, &QPushButton::clicked, onNextButtonClicked);
    QObject::connect(button2, &QPushButton::clicked, onPrevButtonClicked);
    QObject::connect(exitButton, &QPushButton::clicked, [&app]() { onExitButtonClicked(app); });

    // Create a layout for the sliders
    QVBoxLayout *slidersLayout = new QVBoxLayout();
    slidersLayout->addWidget(sprocketSliderLabel);
    slidersLayout->addWidget(sprocketSliderWidget);
    slidersLayout->addWidget(xOffsetSliderLabel);
    slidersLayout->addWidget(xOffsetSliderWidget);
    slidersLayout->addWidget(yOffsetSliderLabel);
    slidersLayout->addWidget(yOffsetSliderWidget);

    // Create a layout for the video and sliders
    QVBoxLayout *videoAndSlidersLayout = new QVBoxLayout();
    videoAndSlidersLayout->addWidget(videoLabel, 0, Qt::AlignCenter); // Center the video label
    videoAndSlidersLayout->addLayout(slidersLayout);

    // Create a layout for the buttons
    QVBoxLayout *buttonsLayout = new QVBoxLayout();
    buttonsLayout->addWidget(button1);
    buttonsLayout->addWidget(button2);
    buttonsLayout->addWidget(exitButton);
    buttonsLayout->addStretch(); // Add stretch to push buttons to the top

    // Create a horizontal layout for the main content
    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(videoAndSlidersLayout);
    mainLayout->addLayout(buttonsLayout);

    // Set the layout for the main window
    window.setLayout(mainLayout);

    // Set up a timer to call the state machine
    timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, []() {
        state_machine(); // Call the state machine function
    });
    timer->start(20); // 50 FPS

    // Show the window
    window.show(); // Show the application window (not full screen)

    // Execute the application
    int result = app.exec();

    delete marlin;
    // Stop the camera when the application exits
    cam.stopVideo();

    return result;
}