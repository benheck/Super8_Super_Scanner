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
#include <QLineEdit>
#include <QDoubleValidator>
#include <QFileDialog>
#include <QMessageBox>

#include <iostream>
#include "MarlinController.h"
#include <thread>
#include <atomic>
#include <mutex>

#include <chrono>
#include <iomanip>


MarlinController *marlin = nullptr;

enum states {
    preview,
    switchToSetup,      //Switches mode and grabs 4k base frame that we draw on top of
    setup,              //When not in the scan loop
    scanning,
    frameRewind,
    frameAdvance,
};

states state = setup; // Initialize the current state

int frameSizeX = 4056;      // Width of the camera frame
int frameSizeY = 3040;    // Height of the camera frame
int previewWindowSizeX = frameSizeX / 4; // Size of the preview window
int previewWindowSizeY = frameSizeY / 4; // Size of the preview window

cv::Mat image, gray, binary;
std::mutex cameraMutex; // Mutex to protect shared resources
cv::Mat image4kin;
bool holeFound = false;
int centerX;                //Sprocket center position in pixels
int centerY;
int validYrangeTop = (frameSizeY / 2) - (frameSizeY / 8); // Top of the valid Y range for the sprocket hole
int validYrangeBottom = (frameSizeY / 2) + (frameSizeY / 8); // Bottom of the valid Y range for the sprocket hole
bool findHoleFlag = false;
int validYrangeTopNarrow = (frameSizeY / 2) - (frameSizeY / 32); // Top of the valid Y range for the sprocket hole
int validYrangeBottomNarrow = (frameSizeY / 2) + (frameSizeY / 32); // Bottom of the valid Y range for the sprocket hole
lccv::PiCamera cam; // Global camera object

QLabel *videoLabel = nullptr; // Global reference to the video label
QPushButton *toggleWhiteBalanceButton = nullptr; // Global QLabel for white balance mode
QTimer *timer = nullptr; // Global pointer to the QTimer
QString defaultFilename = "frame"; // Default filename for exports
QString saveFolder = "export"; // Default save folder

int sprocketLeftSide = 700; // Left side of the sprocket hole
int sprocketSafeCenter = 900; // Center of the sprocket hole, to the right of the left side
int sprocketThreshold = 220; // Threshold for sprocket hole detection (200 + this max 255)

// Configurable
bool focusMode = false; // Flag to indicate if focus mode is enabled
bool negative = false; // Flag to indicate if the image is negative
bool filmDirection = true;              // True for forward, false for backward (only disable/enable motors when it flips)
float film_thickness_mm = 0.15f;        // Film thickness per wrap
float super8frameHeight = 4.234f;
float base_spool_diameter = 31.3f;  //Diameter "pully" Marlin is configed to
float movement_mm_turn_target = base_spool_diameter * M_PI; // mm per rotation of the spool
float spool_diameter_mm = 31.3f;         // Set this to your empty take-up spool diameter
float movement_mm_turn;
float mmPerFrame;
float frameDelta = 0.0f;    //How close the sprocket got to center 360
float frameDeltaMax = 0.0f; // Maximum frame delta for the current scan
float frameDeltaMin = 0.0f; // Minimum frame delta for the current scan
float focusScore = 0.0f; // Focus score for the image
int frameNumber = 0; // Frame number for the current scan
WhiteBalance_Modes currentWhiteBalanceMode = WB_INDOOR; // Default to indoor
bool autoExposureOn = true; // Flag to indicate if auto exposure is locked
bool stopScanFlag = false; // Flag to indicate if the scan should be stopped

cv::Point scanSize;             // Initialize scan size point
int scanCropWidthXMax = 3200; // Maximum width for cropping the scan (from leftSprocket to the right)
int scanCropHeightYMax = 1900; // Maximum height for cropping the scan (total with centerY of sprocket in center)

void logWithTimestamp(const std::string &message) {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::cout << "[" << std::put_time(std::localtime(&now_time_t), "%H:%M:%S")
              << "." << std::setfill('0') << std::setw(3) << now_ms.count() << "] "
              << message << std::endl;
}

void showPopup(const QString &message) {
    QMessageBox popup;
    popup.setWindowTitle("Information");
    popup.setText(message);
    popup.setIcon(QMessageBox::Information);
    popup.setStandardButtons(QMessageBox::Ok);
    popup.exec(); // Show the popup
}

QImage matToQImage(const cv::Mat &mat) {// Function to convert cv::Mat to QImage
    if (mat.empty()) {
        return QImage();
    }
    cv::Mat rgb;
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB); // Convert BGR to RGB
    return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}

std::string stateToString(states state) {
    switch (state) {
        case preview: return "Preview";
        case setup: return "Setup";
        case scanning: return "Scanning";
        case frameRewind: return "Frame Rewind";
        case frameAdvance: return "Frame Advance";
        default: return "Unknown State";
    }
}

void drawText(cv::Mat &imageRef, const std::string &text, int xPos, int yPos, double fontScale = 1.0, int thickness = 2) {

    cv::Point position(xPos, yPos); // Position for the text
    const cv::Scalar &textColor = cv::Scalar(255, 255, 255); // White text color
    cv::putText(imageRef, text, position, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 0), thickness * 2);
    cv::putText(imageRef, text, position, cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness);

}

QLabel* createLabel(const QString &text, QWidget *parent = nullptr) {
    QLabel *label = new QLabel(text, parent);
    label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter); // Default alignment
    label->setStyleSheet("color: white; font-size: 12px;"); // Default style
    return label;
}

void onSprocketLetSideSliderChanged(int value) {// Function to handle sprocket left side slider changes
    sprocketLeftSide = value;

    if (sprocketLeftSide > sprocketSafeCenter) { // Ensure the left side is not greater than the safe center
        sprocketLeftSide = sprocketSafeCenter - 1;
    }

}

void onSprocketRightEdgeSliderChanged(int value) {// Function to handle sprocket right edge slider changes
    sprocketSafeCenter = value;

    if (sprocketSafeCenter < sprocketLeftSide) { // Ensure the safe center is not less than the left side
        sprocketSafeCenter = sprocketLeftSide + 1;
    }

}

void onSprocketThresholdSliderChanged(int value) {// Function to handle sprocket threshold slider changes
    sprocketThreshold = value + 200;
}

void onLightOnButtonClicked() {
    marlin->sendGCode("M42 P205 S255"); // Turn on the light (replace with your pin number)
}

void onLightOffButtonClicked() {
    marlin->sendGCode("M42 P205 S0"); // Turn off the light (replace with your pin number)
}

void onFanOnButtonClicked() {
    marlin->sendGCode("M106 S255"); // Turn on the fan (replace with your pin number)
}

void onFanOffButtonClicked() {
    marlin->sendGCode("M107"); // Turn off the fan (replace with your pin number)
}

void onEnableMotorsClicked() {
    marlin->enableMotors(); // Enable all motors
}

void onDisableMotorsClicked() {
    marlin->disableMotors(); // Disable all motors
}

void compute_move_per_frame() {

    float framesPerTurn = (spool_diameter_mm * M_PI) / super8frameHeight; // Frames per rotation of the spool
    
    movement_mm_turn = 0.0f;
    movement_mm_turn_target = base_spool_diameter * M_PI; // mm per rotation of the spool

    mmPerFrame = movement_mm_turn_target / framesPerTurn; // mm per frame of film

}

float computeFocusScore(const cv::Mat &image) {
    // Extract a strip of pixels from the center of the image
    int stripHeight = 100; // Height of the strip
    int centerY = image.rows / 2;
    cv::Rect centerStrip(0, centerY - stripHeight / 2, image.cols, stripHeight);
    cv::Mat strip = image(centerStrip);

    // Convert the strip to grayscale if it's not already
    cv::Mat grayStrip;
    if (strip.channels() == 3) {
        cv::cvtColor(strip, grayStrip, cv::COLOR_BGR2GRAY);
    } else {
        grayStrip = strip;
    }

    // Apply the Laplacian operator to detect edges
    cv::Mat laplacian;
    cv::Laplacian(grayStrip, laplacian, CV_64F);

    // Compute the variance of the Laplacian
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    double variance = stddev[0] * stddev[0];

    // Return the focus score (lower is better focus)
    return static_cast<float>(variance);
}

void cycleWhiteBalanceMode() {

    if (state != preview) {
        std::cout << "White balance mode can only be toggled in preview mode." << std::endl;
        return; // Only allow toggling in preview mode
    }

    // Cycle through the white balance modes
    switch (currentWhiteBalanceMode) {
        case WB_INCANDESCENT:
            currentWhiteBalanceMode = WB_TUNGSTEN;
            break;
        case WB_TUNGSTEN:
            currentWhiteBalanceMode = WB_FLUORESCENT;
            break;
        case WB_FLUORESCENT:
            currentWhiteBalanceMode = WB_INDOOR;
            break;
        case WB_INDOOR:
            currentWhiteBalanceMode = WB_DAYLIGHT;
            break;
        case WB_DAYLIGHT:
            currentWhiteBalanceMode = WB_CLOUDY;
            break;
        case WB_CLOUDY:
            currentWhiteBalanceMode = WB_INCANDESCENT;
            break;
    }

    // Update the QLabel with the current white balance mode
    if (toggleWhiteBalanceButton) {
        QString wbText;
        switch (currentWhiteBalanceMode) {
            case WB_INCANDESCENT: wbText = "Incandescent"; break;
            case WB_TUNGSTEN: wbText = "Tungsten"; break;
            case WB_FLUORESCENT: wbText = "Fluorescent"; break;
            case WB_INDOOR: wbText = "Indoor"; break;
            case WB_DAYLIGHT: wbText = "Daylight"; break;
            case WB_CLOUDY: wbText = "Cloudy"; break;
        }
        toggleWhiteBalanceButton->setText("White Balance: " + wbText);
    }

    cam.stopVideo(); // Stop the video stream
    cam.options->setWhiteBalance(currentWhiteBalanceMode); // Apply the selected mode
    cam.startVideo(); // Restart the video stream

    // Log the current mode
    std::cout << "White Balance Mode Toggled: " << currentWhiteBalanceMode << std::endl;
}

void reset_spool_tracker(QLineEdit *spoolDiameterInput) {
    // Read the value from the text entry field
    QString inputText = spoolDiameterInput->text();
    if (!inputText.isEmpty()) {
        float newDiameter = inputText.toFloat();

        if (newDiameter > 0.0f) {
            spool_diameter_mm = newDiameter; // Update the spool diameter
            std::cout << "Spool diameter updated to: " << spool_diameter_mm << " mm" << std::endl;
        } else {
            std::cerr << "Invalid spool diameter entered. Using default value: " << spool_diameter_mm << " mm" << std::endl;
        }
    } else {
        std::cerr << "No spool diameter entered. Using default value: " << spool_diameter_mm << " mm" << std::endl;
    }

    frameNumber = 0;

    compute_move_per_frame();

    // Reset the total movement tracker
    //data.total_y_movement_mm = 0.0f;
    std::cout << "Spool tracker reset. Total movement set to 0.0 mm." << std::endl;
}

void checkForwardDirection() {      //Forward movement, change direction if needed

    if (filmDirection != true) {
        marlin->disableSingleMotor("X"); // Disable the X motor feed
        marlin->enableSingleMotor("Y"); // Enable the Y motor takeup
        filmDirection = true; // Set the direction to forward
    }

}

void checkBackwardDirection() {     //Backward movement, change direction if needed

    if (filmDirection != false) {
        marlin->disableSingleMotor("Y"); // Disable the Y motor takeup
        marlin->enableSingleMotor("X"); // Enable the X motor feed
        filmDirection = false; // Set the direction to backward
    }

}

//Send scaled commands to Marlin and keep track of spool diameters which affect movement per frame
void advanceFilmTracking(float frames, float speed, bool waitForResponse = false) {

    float mm = frames * mmPerFrame; // Convert frames to mm using the mm per frame value
    marlin->advanceFilm(mm, speed, waitForResponse); // Advance the film by mm
    movement_mm_turn += mm;     // Update the total movement tracker

    //Add multiple-turn stacking
    if (movement_mm_turn >= movement_mm_turn_target) {
        float leftover = movement_mm_turn - movement_mm_turn_target;  //Save this

        spool_diameter_mm += (film_thickness_mm * 2);   // Increase the spool diameter by twice the film thickness

        compute_move_per_frame();       // Recompute the movement per frame based on diameter
        movement_mm_turn = leftover;    // Reset the movement to the leftover value
    }

}

void rewindFilmTracking(float frames, float speed, bool waitForResponse = false) {

}

//For film transport buttons
void shuttleFilmForward(float frames, float speed, bool waitForMoveComplete = false) {
    checkForwardDirection(); // Check and set the direction to forward
    advanceFilmTracking(frames, speed, waitForMoveComplete); // Call the overloaded function with waitForResponse = true
}

cv::Mat scaleToFitFrameSize(const cv::Mat &inputMat, int targetWidth) {
    if (inputMat.empty()) {
        std::cerr << "Error: Input Mat is empty!" << std::endl;
        return cv::Mat();
    }

    // Calculate the scaling factor to fit the target width
    double scale = static_cast<double>(targetWidth) / inputMat.cols;

    // Calculate the new dimensions while maintaining the aspect ratio
    int newWidth = targetWidth;
    int newHeight = static_cast<int>(inputMat.rows * scale);

    // Resize the input Mat to the new dimensions
    cv::Mat resizedMat;
    cv::resize(inputMat, resizedMat, cv::Size(newWidth, newHeight));

    //Center image vertically in a window sized previewWindowSizeX x previewWindowSizeY:
    int yOffset = (previewWindowSizeY - newHeight) / 2; // Calculate the vertical offset
    cv::Mat centeredMat(previewWindowSizeY, previewWindowSizeX, CV_8UC3, cv::Scalar(0, 0, 0)); // Create a black image of the target size
    resizedMat.copyTo(centeredMat(cv::Rect(0, yOffset, newWidth, newHeight))); // Copy the resized image to the centered position
    
    return centeredMat;

}

cv::Mat focusModeDisplay(const cv::Mat &inputMat) {

    // Calculate the crop rectangle to center the image
    int startX = (inputMat.cols - previewWindowSizeX) / 2; // Center horizontally
    int startY = (inputMat.rows - previewWindowSizeY) / 2; // Center vertically

    // Crop the center of the image
    cv::Rect cropRect(startX, startY, previewWindowSizeX, previewWindowSizeY);

    return inputMat(cropRect);

}

void drawPreviewModeStatus(cv::Mat &imageRef) {

    drawText(imageRef, "STATE: " + stateToString(state), 10, 40, 1.0); // Draw the current state on the image
    drawText(imageRef, "Sprocket XY @ 720p: " + std::to_string(centerX) + " " + std::to_string(centerY), 10, 80, 1.0); // Draw the sprocket coordinates on the image
    
    
    
    
    drawText(imageRef, "Sprocket last error delta: " + std::to_string(frameDelta) + "%", 10, 120, 1.0); // Draw the sprocket error delta on the image

    drawText(imageRef, "Takeup spool diameter: " + std::to_string(spool_diameter_mm) + " mm", 10, 240, 1.0); // Draw the takeup spool diameter on the image
    drawText(imageRef, "Takeup movement per frame: " + std::to_string(mmPerFrame) + " mm", 10, 280, 1.0); // Draw the takeup movement per frame on the image
    
    drawText(imageRef, "Movement per Turn: " + std::to_string(movement_mm_turn) + " mm", 10, 320, 1.0); // Draw the movement per turn on the image
    drawText(imageRef, "Movement Target: " + std::to_string(movement_mm_turn_target) + " mm", 10, 360, 1.0); // Draw the movement target on the image



}

void switchToPreviewMode() {        //Run fast, scaled live video

    if (state == preview) {
        return; // If already in preview mode, do nothing
    }

    cam.stopPhoto(); // Stop the photo mode if it's running

    state = preview; // Set the state to preview mode

    if (!cam.startVideo()) {
        std::cerr << "Error: Failed to start video!" << std::endl;
        exit(-1); // Exit if the camera fails to start
    }

    std::cout << "Switched to preview mode." << std::endl;

}

void switchToSetupMode() {      //Capture full sensor images

    if (state == setup) {
        return; // If already in setup mode, do nothing
    }

    cam.stopVideo(); // Stop the video mode if it's running
    cam.startPhoto(); // Start the photo mode
    state = switchToSetup;    // Grab still frame then jump to setup

    std::cout << "Switched to setup mode." << std::endl;

}

void updateVideoFeed() {

    if (!cam.getVideoFrame(image, 1000)) { // Capture a video frame from the camera
        std::cerr << "Error: Failed to capture video frame!" << std::endl;
        return;
    }

    cv::flip(image, image, 1); // Flip the image horizontally

    //Do focus mode and draw shit on the image, or just preview mode
    if (focusMode) {
        image = focusModeDisplay(image); // Crop the image to the center for focus mode
        drawText(image, "STATE: " + stateToString(state) + " / FOCUSING", 10, 40, 1.0); // Draw the current state on the image
        drawText(image, "Focus Score: " + std::to_string(computeFocusScore(image)) + " higher is better", 10, previewWindowSizeY - 40, 1.0); // Draw the focus score on the image
    }
    else {
        image = scaleToFitFrameSize(image, previewWindowSizeX); // Scale the image to fit the preview window size 

        //Draw center bombsight
        cv::line(image, cv::Point(previewWindowSizeX / 2, 0), cv::Point(previewWindowSizeX / 2, previewWindowSizeY), cv::Scalar(255, 0, 0), 5);
        cv::line(image, cv::Point(0, previewWindowSizeY / 2), cv::Point(previewWindowSizeX, previewWindowSizeY / 2), cv::Scalar(255, 0, 0), 5);

        //Draw info on screen TODO: Add more!
        drawPreviewModeStatus(image); 

    }

    QImage qImage = matToQImage(image);

    if (!qImage.isNull() && videoLabel != nullptr) {
        videoLabel->setPixmap(QPixmap::fromImage(qImage)); // Display the frame in QLabel
    }

}

void updateSetupImage() {       //Draws on a frame buffer for SPEED
    
    image = image4kin.clone(); // Clone the 4k image to the image variable

    cv::flip(image, image, 1);      // Flip hori

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);    // Convert the image to grayscale
    cv::threshold(gray, gray, sprocketThreshold, 255, cv::THRESH_BINARY); // Apply binary thresholding to isolate the white sprocket hole

    //Crop binary to be same width as sprocket hole safe center:
    cv::Rect cropRect(sprocketLeftSide, 0, sprocketSafeCenter - sprocketLeftSide, frameSizeY); // Define the crop rectangle
    cv::Mat croppedBinary = gray(cropRect); // Crop the binary image

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(croppedBinary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // Find contours

    holeFound = false; // Reset hole found flag

        // Draw a vertical line at the left side of the sprocket hole
        cv::line(image, cv::Point(sprocketLeftSide, 0), cv::Point(sprocketLeftSide, frameSizeY), cv::Scalar(255, 0, 0), 5);

        // Draw a vertical line for sprocket safe center (slightly left of where vertical tears might be)
        cv::line(image, cv::Point(sprocketSafeCenter, 0), cv::Point(sprocketSafeCenter, frameSizeY), cv::Scalar(0, 0, 255), 5);

    for (const auto& contour : contours) {

        cv::Rect boundingBox = cv::boundingRect(contour);

        //Box must be in middle 1/3rd of image vertically:
        if (boundingBox.y > (image.rows / 3) && boundingBox.y < ((image.rows / 3) * 2)) {    //Must be center 3rd of image vertically

            if (boundingBox.height > 300) {

                holeFound = true; // Set the hole found flag to true
    
                //Move bounding box sprocketLeftSide pixels right:
                boundingBox.x += sprocketLeftSide; // Move the bounding box to the right by sprocketLeftSide pixels
    
                //Calc center of boundingBox:
                centerX = boundingBox.x + (boundingBox.width / 2);
                centerY = boundingBox.y + (boundingBox.height / 2);
    
                // Draw a green rectangle around the detected sprocket hole:
                cv::rectangle(image, boundingBox, cv::Scalar(0, 255, 0), 10);
    
                break; // Stop after detecting the first sprocket hole
    
            }

        }


    }

    if (holeFound) {

        //Draw a yellow rectangle from sprocketLeftSide to scanCropWidthXMax, with height scanCropHeightYMax center on centerY:
        cv::rectangle(image, cv::Point(sprocketSafeCenter, centerY - (scanCropHeightYMax / 2)), cv::Point(scanCropWidthXMax, centerY + (scanCropHeightYMax / 2)), cv::Scalar(255, 255, 0), 6);

        //Compute the scan size of the above rectangle:
        scanSize.x = scanCropWidthXMax - sprocketSafeCenter; // Width of the scan size rectangle
        scanSize.y = scanCropHeightYMax; // Height of the scan size rectangle

    }

    // Make image 1/4th size for display
    cv::resize(image, image, cv::Size(frameSizeX / 4, frameSizeY / 4));

    drawText(image, "STATE: " + stateToString(state), 10, 40, 1.0); // Draw the current state on the image
    drawText(image, "Scan frame size: " + std::to_string(scanSize.x) + " x " + std::to_string(scanSize.y), 10, 80, 1.0); // Draw the scan frame size on the image

    QImage qImage = matToQImage(image); // Convert cv::Mat to QImage

    if (!qImage.isNull() && videoLabel != nullptr) {
        videoLabel->setPixmap(QPixmap::fromImage(qImage)); // Display the frame in QLabel
    }

}

void scanSingleFrame() {        //Like updateSetupImage but for scanning

    //Log frame time:
    auto startTime = std::chrono::high_resolution_clock::now(); // Start time for logging

    if (cam.capturePhoto(image) == false) {
        std::cerr << "Error: Failed to capture burner photo!" << std::endl;
        return; // Skip if the image capture fails
    }  

    if (cam.capturePhoto(image) == false) {
        std::cerr << "Error: Failed to capture real photo!" << std::endl;
        return; // Skip if the image capture fails
    }  

    cv::flip(image, image, 1);      // Flip hori

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);    // Convert the image to grayscale
    cv::threshold(gray, gray, sprocketThreshold, 255, cv::THRESH_BINARY); // Apply binary thresholding to isolate the white sprocket hole

    // //Crop binary to be same width as sprocket hole safe center:
    cv::Rect cropRect(sprocketLeftSide, 0, sprocketSafeCenter - sprocketLeftSide, frameSizeY); // Define the crop rectangle
    cv::Mat croppedBinary = gray(cropRect); // Crop the binary image

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(croppedBinary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // Find contours

    holeFound = false; // Reset hole found flag this frame

    int toleranceYtop = validYrangeTop;
    int toleranceYbottom = validYrangeBottom; // Bottom of the valid Y range for the sprocket hole

    if (findHoleFlag) {     //Lost last frame? Higher tolerance required
        toleranceYtop = validYrangeTopNarrow; // Top of the valid Y range for the sprocket hole
        toleranceYbottom = validYrangeBottomNarrow; // Bottom of the valid Y range for the sprocket hole
    }

    for (const auto& contour : contours) {

        cv::Rect boundingBox = cv::boundingRect(contour);

        //Calc center of boundingBox:
        centerX = boundingBox.x + (boundingBox.width / 2);
        centerY = boundingBox.y + (boundingBox.height / 2);

        if (centerY > toleranceYtop && centerY < toleranceYbottom) {  //Must be in center 4th vertically

            if (boundingBox.height > 300) {

                holeFound = true; // Set the hole found flag to true

                if (findHoleFlag) { // If the hole is found, set the flag to false
                    findHoleFlag = false; // Reset the flag
                    std::cout << "Sprocket hole re-acquired" << std::endl; // Log the event
                }

                //Move bounding box sprocketLeftSide pixels right:
                boundingBox.x += sprocketLeftSide; // Move the bounding box to the right by sprocketLeftSide pixels
                centerX += sprocketLeftSide; // Move the centerX to the right by sprocketLeftSide pixels

                // Draw a green rectangle around the detected sprocket hole:
                cv::rectangle(image, boundingBox, cv::Scalar(0, 255, 0), 10);
    
                break; // Stop after detecting the first sprocket hole
    
            }

        }


    }

    // Draw a vertical line at the left side of the sprocket hole
    cv::line(image, cv::Point(sprocketLeftSide, 0), cv::Point(sprocketLeftSide, frameSizeY), cv::Scalar(255, 0, 0), 5);

    // Draw a vertical line for sprocket safe center (slightly left of where vertical tears might be)
    cv::line(image, cv::Point(sprocketSafeCenter, 0), cv::Point(sprocketSafeCenter, frameSizeY), cv::Scalar(0, 0, 255), 5);

    if (holeFound) {

        //Crop image the size as this rectangle:

        if (centerY - (scanCropHeightYMax / 2) < 0) { // Check if the centerY is less than half the crop height
            
            logWithTimestamp("Top of crop < 0 Y, nudging...");
            shuttleFilmForward(0.05f, 250.0f); // Move the film forward by one frame

        }
        else {

            cv::Rect cropRect(sprocketSafeCenter, centerY - (scanCropHeightYMax / 2), scanCropWidthXMax - sprocketSafeCenter, scanCropHeightYMax); // Define the crop rectangle
            cv::Mat croppedPhoto = image(cropRect); // Crop the photo

            //Calculate frameDelt as a float between -1 and 1, based on how far above or below centerY is to 1520 with a frameSizeY of 3040:
            frameDelta = ((float)(frameSizeY / 2) - (float)centerY) / (float)(frameSizeY / 2); // Calculate the frame delta as a float between -1 and 1

            advanceFilmTracking(1.0f - frameDelta, 2000.0f);
    
            // Save the photo to a file with the frame number (bitmap is fastest)
            std::ostringstream oss;
            oss << std::setw(5) << std::setfill('0') << frameNumber; // Format the frame number with leading zeros
            std::string formattedFrameNumber = oss.str();
            std::string filename = saveFolder.toStdString() + "/" + defaultFilename.toStdString() + "_" + formattedFrameNumber + ".png";
            cv::imwrite(filename, croppedPhoto); // Save the photo to the selected folder
    
            frameNumber++;

        }

    }
    else {

        //Check center of screen for empty image:
        cv::Rect cropRect(sprocketSafeCenter, (frameSizeY / 2) - (scanCropHeightYMax / 2), scanCropWidthXMax - sprocketSafeCenter, scanCropHeightYMax); // Define the crop rectangle
        cv::Mat croppedPhoto = image(cropRect); // Crop the photo
        cv::Scalar meanValue = cv::mean(croppedPhoto); // Calculate the mean value of the cropped photo

        if (meanValue[0] > 240) {           // Check if the mean value is above a certain threshold (indicating a mostly empty image)
            stopScanFlag = true;            // Set the stop scan flag to true

            onLightOffButtonClicked();      // Turn off the light
            onFanOffButtonClicked();        // Turn off the fan
            onDisableMotorsClicked();       // Disable the motors
            switchToPreviewMode(); // Switch back to preview mode

            showPopup("Scan completed"); // Show a popup message indicating the scan is complete

            return;                         // Exit the function if the image is mostly empty
        }

        findHoleFlag = true; // Find hole next frames(s) higher tolerance

        logWithTimestamp("No sprocket hole found, nudging...");

        shuttleFilmForward(0.05f, 250.0f); // Move the film forward by one frame

    }

    cv::line(image, cv::Point(0, toleranceYtop), cv::Point(frameSizeX, toleranceYtop), cv::Scalar(255, 255, 0), 5);
    cv::line(image, cv::Point(0, toleranceYbottom), cv::Point(frameSizeX, toleranceYbottom), cv::Scalar(255, 255, 0), 5);

    // Make image 1/4th size for display
    cv::resize(image, image, cv::Size(frameSizeX / 4, frameSizeY / 4));

    drawText(image, "STATE: " + stateToString(state), 10, 40, 1.0); // Draw the current state on the image
    drawText(image, "Frame Delta: " + std::to_string(frameDelta) + "%", 10, 80, 1.0); // Draw the sprocket error delta on the image

    if (frameDelta > frameDeltaMax) {
        frameDeltaMax = frameDelta; // Update the maximum frame delta
    }
    if (frameDelta < frameDeltaMin) {
        frameDeltaMin = frameDelta; // Update the minimum frame delta
    }

    drawText(image, "Max Delta: " + std::to_string(frameDeltaMax) + "%", 10, 120, 1.0); // Draw the maximum frame delta on the image
    drawText(image, "Min Delta: " + std::to_string(frameDeltaMin) + "%", 10, 160, 1.0); // Draw the minimum frame delta on the image

    QImage qImage = matToQImage(image); // Convert cv::Mat to QImage

    if (!qImage.isNull() && videoLabel != nullptr) {
        videoLabel->setPixmap(QPixmap::fromImage(qImage)); // Display the frame in QLabel
    }

    //marlin->waitForMoveCompletion(); // Wait for the move to complete

    auto endTime = std::chrono::high_resolution_clock::now(); // End time for logging

    //Print frame time:
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count(); // Calculate the duration in milliseconds
    std::cout << "Frame " << frameNumber << " time: " << duration << " ms" << std::endl;
   

}

void onStartScanButtonClicked() {

    //Make a popup window if currentState is not setup:
    if (state != setup) {
        showPopup("Scan must be started in setup mode");
        return; // Exit if the scan is already in progress
    }

    state = scanning; // Set the current state to scanning
    stopScanFlag = false; // Reset the stop scan flag
    frameDeltaMax = 0.0f; // Reset the maximum frame delta
    frameDeltaMin = 0.0f; // Reset the minimum frame delta

    //cam.startPhoto(); // Start the photo mode
    scanSingleFrame();          // Capture first photo   

}

void onStopScanButtonClicked() {

    if (state == scanning) {
        stopScanFlag = true;
    }

}

void state_machine() {

    try
    { 
        switch(state) {
            case preview:     //Runs full speed video
                updateVideoFeed(); // Update the video feed
                break;
    
            case switchToSetup:
                if (cam.capturePhoto(image4kin) == false) {
                    std::cerr << "Error: Failed to capture photo!" << std::endl;
                    return; // Skip if the image capture fails
                }        
    
                state = setup; // Switch to setup state
    
                break;
    
            case setup:                 //Still images and sprocket calibration
                updateSetupImage(); // Update the setup image
                break;
    
            case scanning:              //Scanning loop
                if (stopScanFlag) {
                    stopScanFlag = false;       // Reset the stop scan flag
                    switchToPreviewMode(); // Switch back to preview mode
                }
                else {
                    scanSingleFrame();          // Capture a single frame for scanning
                }
    
                break;
    
        }
    }
    catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;

        onLightOffButtonClicked();      // Turn off the light
        onFanOffButtonClicked();        // Turn off the fan

        return; // Skip if the update fails
    }


}

void cameraSetup() {
    uint32_t num_cams = LibcameraApp::GetNumberCameras();
    std::cout << "Found " << num_cams << " cameras." << std::endl;

    cam.options->video_width = 1920;
    cam.options->video_height = 1080;
    cam.options->framerate = 30;
    cam.options->photo_width = 4056;
    cam.options->photo_height = 3040;
    cam.options->denoise = "off";

}

void onExitButtonClicked(QApplication &app) {
    std::cout << "Exit button clicked!" << std::endl;

    // Stop the QTimer
    timer->stop();
    std::cout << "QTimer stopped." << std::endl;

    onLightOffButtonClicked();      // Turn off the light
    onFanOffButtonClicked();        // Turn off the fan

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

void printCameraSettings() {
    std::cout << "Current Camera Settings:" << std::endl;
    std::cout << "  Exposure Mode: " << cam.options->getExposureMode() << std::endl;
    std::cout << "  Shutter (Exposure Time): " << cam.options->shutter << " seconds" << std::endl;
    std::cout << "  Gain: " << cam.options->gain << std::endl;
    std::cout << "  Brightness: " << cam.options->brightness << std::endl;
    std::cout << "  Contrast: " << cam.options->contrast << std::endl;
    std::cout << "  Saturation: " << cam.options->saturation << std::endl;
    std::cout << "  Sharpness: " << cam.options->sharpness << std::endl;
    std::cout << "  White Balance Mode: " << cam.options->getWhiteBalance() << std::endl;
    std::cout << "  Exposure Compensation (EV): " << cam.options->ev << std::endl;
    std::cout << "  Metering Mode: " << cam.options->getMeteringMode() << std::endl;
    std::cout << "  Denoise: " << cam.options->denoise << std::endl;
    std::cout << "  Photo Width: " << cam.options->photo_width << std::endl;
    std::cout << "  Photo Height: " << cam.options->photo_height << std::endl;
    std::cout << "  Video Width: " << cam.options->video_width << std::endl;
    std::cout << "  Video Height: " << cam.options->video_height << std::endl;

}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    try {
        marlin = new MarlinController("/dev/ttyACM0", 115200); // Replace with your serial port and baud rate
    } catch (const std::exception &e) {
        std::cerr << "Error initializing Marlin controller: " << e.what() << std::endl;
        return -1;
    }

    //If anything is mounted in /media, set saveFolder to that path:
    QDir dir("/media");
    if (dir.exists()) {
        QStringList drives = dir.entryList(QDir::NoDotAndDotDot | QDir::AllDirs);
        if (!drives.isEmpty()) {
            saveFolder = "/media/" + drives[0]; // Set the save folder to the first mounted drive
        }
    }
    if (saveFolder.isEmpty()) {
        saveFolder = QDir::homePath(); // Default to home directory if no drives are found
    }

    //Default is an empty take-up spool of 31.3mm diameter but you can change this after boot
    spool_diameter_mm = base_spool_diameter; // Set the spool diameter to the base diameter
    compute_move_per_frame(); //Calc default takeup spool (empty default)

    cameraSetup(); // Set up the camera
    cam.options->setWhiteBalance(currentWhiteBalanceMode); // Apply the selected mode
    switchToPreviewMode(); // Switch to preview mode to start

    QWidget window;
    window.setWindowTitle("Super 8mm Super Scanner");
    window.resize(1800, 900); // Set the window size to 1800x900
    window.setStyleSheet("background-color: #333333;"); // Dark gray background

    // Create a horizontal layout for the main content
    QHBoxLayout *mainLayout = new QHBoxLayout();

    // Create a layout for the video and sliders left half of window------------------------------
    QVBoxLayout *videoAndSlidersLayout = new QVBoxLayout();

    // Create a video display label
    videoLabel = new QLabel();
    videoLabel->setFixedSize(frameSizeX / 4, frameSizeY / 4); // Set video frame display size
    videoLabel->setStyleSheet("background-color: black;"); // Black background for video

    videoAndSlidersLayout->addWidget(videoLabel, 0, Qt::AlignCenter); // Center the video label upper left

    // Create a grid layout for the sliders
    QGridLayout *slidersGridLayout = new QGridLayout();
    slidersGridLayout->setAlignment(Qt::AlignTop); // Align the grid layout to the top

      //Column 0 row 0-1:
    QSlider *leftSprocketSliderWidget = new QSlider(Qt::Horizontal);
    leftSprocketSliderWidget->setRange(0, 1500);
    leftSprocketSliderWidget->setValue(sprocketLeftSide); // Set the initial value to 0
    QObject::connect(leftSprocketSliderWidget, &QSlider::valueChanged, onSprocketLetSideSliderChanged);

    slidersGridLayout->addWidget(createLabel("Left side of sprocket"), 0, 0); // Add a label for the left sprocket slider
    slidersGridLayout->addWidget(leftSprocketSliderWidget, 1, 0); // Add the slider to the grid layout

       //Column 0 row 2-3:
    QSlider *rightSprocketSliderWidget = new QSlider(Qt::Horizontal);
    rightSprocketSliderWidget->setRange(0, 1500);
    rightSprocketSliderWidget->setValue(sprocketSafeCenter); // Set the initial value to 0
    QObject::connect(rightSprocketSliderWidget, &QSlider::valueChanged, onSprocketRightEdgeSliderChanged);

    slidersGridLayout->addWidget(createLabel("Sprocket safe center"), 2, 0); // Add a label for the left sprocket slider
    slidersGridLayout->addWidget(rightSprocketSliderWidget, 3, 0); // Add the slider to the grid layout

    //Column 0 row 4-5:
    QSlider *sprocketThresholdSliderWidget = new QSlider(Qt::Horizontal);
    sprocketThresholdSliderWidget->setRange(0, 55); // Range for sprocket threshold slider
    sprocketThresholdSliderWidget->setValue(sprocketThreshold - 200); // Set initial value
    QObject::connect(sprocketThresholdSliderWidget, &QSlider::valueChanged, onSprocketThresholdSliderChanged);

    slidersGridLayout->addWidget(createLabel("Sprocket white threshold"), 4, 0); // Add a label for the left sprocket slider
    slidersGridLayout->addWidget(sprocketThresholdSliderWidget, 5, 0); // Add the slider to the grid layout

    //Column 1 row 0-1:
    QSlider *cropWidthSliderWidget = new QSlider(Qt::Horizontal);
    cropWidthSliderWidget->setRange(3000, 4055); // Range for crop width slider
    cropWidthSliderWidget->setValue(scanCropWidthXMax); // Set initial value
    QObject::connect(cropWidthSliderWidget, &QSlider::valueChanged, [](int value) {
        scanCropWidthXMax = value; // Update the global variable when the text changes
    });

    slidersGridLayout->addWidget(createLabel("Scan Crop width"), 0, 1); // Add a label for the left sprocket slider
    slidersGridLayout->addWidget(cropWidthSliderWidget, 1, 1); // Add the slider to the grid layout

    //Column 1 row 2-3:
    QSlider *cropHeightSliderWidget = new QSlider(Qt::Horizontal);
    cropHeightSliderWidget->setRange(1600, 2200); // Range for crop height slider
    cropHeightSliderWidget->setValue(scanCropHeightYMax); // Set initial value
    QObject::connect(cropHeightSliderWidget, &QSlider::valueChanged, [](int value) {
        scanCropHeightYMax = value; // Update the global variable when the text changes
    });

    slidersGridLayout->addWidget(createLabel("Scan Crop height"), 2, 1); // Add a label for the left sprocket slider
    slidersGridLayout->addWidget(cropHeightSliderWidget, 3, 1); // Add the slider to the grid layout

    //Column 1 row 4-5:
    slidersGridLayout->addWidget(createLabel("NEGATIVE SHIT"), 0, 2); // Add a label for the left sprocket slider

    videoAndSlidersLayout->addLayout(slidersGridLayout); // Add the sliders grid layout to the video and sliders layout


    mainLayout->addLayout(videoAndSlidersLayout);           //ADD TO LAYOUT

    //Create film buttons:
    QPushButton *gotoPreviewModeButton = new QPushButton("Preview Mode");
    QPushButton *gotoSetupModeButton = new QPushButton("Setup Mode");
    QPushButton *focusButton = new QPushButton("Focus Mode: Off");
    QPushButton *nudgeButtonForward = new QPushButton("Nudge Forward");
    QPushButton *nudgeButtonBack = new QPushButton("Nudge Backward");
    QPushButton *startScanButton = new QPushButton("Start Scan");
    QPushButton *stopScanButton = new QPushButton("Stop Scan");
    QPushButton *advance1Button = new QPushButton("Frame +1");
    QPushButton *advance10Button = new QPushButton("Frame +10");
    QPushButton *advance50Button = new QPushButton("Frame +50");
    QPushButton *advance100Button = new QPushButton("Frame +100");

    // Connect film transport buttons to their respective functions
    QObject::connect(gotoPreviewModeButton, &QPushButton::clicked, switchToPreviewMode); // Connect to preview mode function
    QObject::connect(gotoSetupModeButton, &QPushButton::clicked, switchToSetupMode); // Connect to setup mode function

    QObject::connect(focusButton, &QPushButton::clicked, [focusButton]() {
        focusMode = !focusMode; // Toggle

        if (focusMode) {
            focusButton->setText("Focus Mode: On");
        } else {
            focusButton->setText("Focus Mode: Off");
        }
    });

    QObject::connect(nudgeButtonForward, &QPushButton::clicked, []() {
        shuttleFilmForward(0.05f, 250.0f); // Move the film forward by one frame
    });

    QObject::connect(advance1Button, &QPushButton::clicked, []() {
        shuttleFilmForward(1.0f + frameDelta, 1000.0f); // Move the film forward by one frame
    });

    QObject::connect(advance10Button, &QPushButton::clicked, []() {
        shuttleFilmForward(10.0f, 1000.0f); // Move the film forward by one frame
    });

    QObject::connect(advance50Button, &QPushButton::clicked, []() {
        shuttleFilmForward(50.0f, 2000.0f); // Move the film forward by one frame
    });

    QObject::connect(advance100Button, &QPushButton::clicked, []() {
        shuttleFilmForward(100.0f, 3000.0f); // Move the film forward by one frame
    });

    QObject::connect(startScanButton, &QPushButton::clicked, onStartScanButtonClicked);
    QObject::connect(stopScanButton, &QPushButton::clicked, onStopScanButtonClicked);

    QLineEdit *filenameInput = new QLineEdit();
    filenameInput->setPlaceholderText("Change base filename");
    //filenameInput->setText(defaultFilename); // Set the default value
    filenameInput->setStyleSheet(
        "color: black;"                // Text color
        "background-color: white;"     // Background color
        "selection-color: white;"      // Selected text color
        "selection-background-color: blue;" // Background color for selected text
        "border: 1px solid gray;"      // Border styling
        "padding: 5px;"                // Padding for better spacing
        "font-size: 14px;"             // Font size for readability
        "caret-color: black;"          // Cursor (caret) color
    );
    QObject::connect(filenameInput, &QLineEdit::textChanged, [](const QString &text) {
        defaultFilename = text; // Update the global variable when the text changes
    });

    QPushButton *selectFolderButton = new QPushButton("Select Save Folder");
    QObject::connect(selectFolderButton, &QPushButton::clicked, []() {
        QString folder = QFileDialog::getExistingDirectory(nullptr, "Select Save Folder", saveFolder);
        if (!folder.isEmpty()) {
            saveFolder = folder; // Update the save folder path
            std::cout << "Save folder updated to: " << saveFolder.toStdString() << std::endl;
        }

    });

    // Create a layout for the buttons
    QVBoxLayout *filmButtonsLayout = new QVBoxLayout();
    filmButtonsLayout->addWidget(gotoPreviewModeButton);
    filmButtonsLayout->addWidget(gotoSetupModeButton);
    filmButtonsLayout->addWidget(focusButton);
    filmButtonsLayout->addWidget(nudgeButtonForward);
    filmButtonsLayout->addWidget(advance1Button);
    filmButtonsLayout->addWidget(advance10Button);
    filmButtonsLayout->addWidget(advance50Button);
    filmButtonsLayout->addWidget(advance100Button);
    filmButtonsLayout->addWidget(startScanButton);
    filmButtonsLayout->addWidget(stopScanButton);
    filmButtonsLayout->addWidget(filenameInput);
    filmButtonsLayout->addWidget(selectFolderButton);

    filmButtonsLayout->addStretch(); // Add stretch to push buttons to the top

    mainLayout->addLayout(filmButtonsLayout); // Add the film buttons layout to the main layout

    // Create control buttons--------------------------------------------------------
    QLineEdit *spoolDiameterInput = new QLineEdit();
    spoolDiameterInput->setPlaceholderText("Starting takeup spool diameter (mm) default 31.3");
    spoolDiameterInput->setStyleSheet(
        "color: black;"                // Text color
        "background-color: white;"     // Background color
        "selection-color: white;"      // Selected text color
        "selection-background-color: blue;" // Background color for selected text
        "border: 1px solid gray;"      // Border styling
        "padding: 5px;"                // Padding for better spacing
        "font-size: 14px;"             // Font size for readability
        "caret-color: black;"          // Cursor (caret) color
    );
    spoolDiameterInput->setValidator(new QDoubleValidator(0.0, 1000.0, 2)); // Allow only valid float values (0.0 to 100.0)
    QPushButton *resetSpoolButton = new QPushButton("Reset Spool");
    QPushButton *negativeButton = new QPushButton("Positive Film");
    toggleWhiteBalanceButton = new QPushButton("White Balance: Indoor");      //Starts here
     QPushButton *lockExposureButton = new QPushButton("Exposure: Auto");   
    QPushButton *lightOnButton = new QPushButton("Light On");
    QPushButton *lightOffButton = new QPushButton("Light Off");
    QPushButton *fanOnButton = new QPushButton("Fan On");
    QPushButton *fanOffButton = new QPushButton("Fan Off");
    QPushButton *enableMotorsButton = new QPushButton("Enable Motors");
    QPushButton *disableMotorsButton = new QPushButton("Disable Motors");
    QPushButton *rewindFilmButton = new QPushButton("Rewind Film");
    QPushButton *stopRewindFilmButton = new QPushButton("Stop Rewind");
    QPushButton *exitButton = new QPushButton("Exit");


    QObject::connect(resetSpoolButton, &QPushButton::clicked, [spoolDiameterInput]() {
        reset_spool_tracker(spoolDiameterInput);
    });

    QObject::connect(negativeButton, &QPushButton::clicked, [negativeButton]() {
        negative = !negative; // Toggle the negative boolean
        if (negative) {
            negativeButton->setText("Negative Film");
        } else {
            negativeButton->setText("Positive Film");
        }
        std::cout << "Negative mode is now " << (negative ? "ON" : "OFF") << std::endl;
    });

    QObject::connect(toggleWhiteBalanceButton, &QPushButton::clicked, []() {
        cycleWhiteBalanceMode(); //TODO: with threading
    });

    QObject::connect(lockExposureButton, &QPushButton::clicked, [lockExposureButton]() {
        autoExposureOn = !autoExposureOn; // Toggle the lock auto-exposure boolean
        if (autoExposureOn) {
            lockExposureButton->setText("Exposure: Auto");
        } else {
            lockExposureButton->setText("Exposure: Locked");
        }
        cam.options->setAutoExposure(autoExposureOn); // Set the auto-exposure lock in the camera options
        std::cout << "Auto-exposure is now " << (autoExposureOn ? "LOCKED" : "UNLOCKED") << std::endl;

    });

    QObject::connect(lightOnButton, &QPushButton::clicked, onLightOnButtonClicked);
    QObject::connect(lightOffButton, &QPushButton::clicked, onLightOffButtonClicked);
    QObject::connect(fanOnButton, &QPushButton::clicked, onFanOnButtonClicked);
    QObject::connect(fanOffButton, &QPushButton::clicked, onFanOffButtonClicked);
    QObject::connect(enableMotorsButton, &QPushButton::clicked, onEnableMotorsClicked);
    QObject::connect(disableMotorsButton, &QPushButton::clicked, onDisableMotorsClicked);

    QObject::connect(rewindFilmButton, &QPushButton::clicked, []() {
        marlin->sendGCode("G1 Y-99999 F18000"); // Send G-code command to rewind film
    });

    QObject::connect(stopRewindFilmButton, &QPushButton::clicked, []() {
        marlin->sendGCode("M410"); // Send G-code command to stop rewinding film
    });

    QObject::connect(exitButton, &QPushButton::clicked, [&app]() {
        onExitButtonClicked(app);
    });

    QVBoxLayout *buttonsSetup = new QVBoxLayout();

    buttonsSetup->addWidget(spoolDiameterInput);
    buttonsSetup->addWidget(resetSpoolButton);
    buttonsSetup->addWidget(negativeButton);
    buttonsSetup->addWidget(toggleWhiteBalanceButton);
    buttonsSetup->addWidget(lockExposureButton);
    buttonsSetup->addWidget(lightOnButton);
    buttonsSetup->addWidget(lightOffButton);
    buttonsSetup->addWidget(fanOnButton);
    buttonsSetup->addWidget(fanOffButton);
    buttonsSetup->addWidget(enableMotorsButton);
    buttonsSetup->addWidget(disableMotorsButton);
    buttonsSetup->addWidget(rewindFilmButton);
    buttonsSetup->addWidget(stopRewindFilmButton);
    buttonsSetup->addWidget(exitButton);
    buttonsSetup->addStretch(); // Add stretch to push buttons to the top

    mainLayout->addLayout(buttonsSetup); // Add the buttons setup layout to the main layout

    // Set the layout for the main window
    window.setLayout(mainLayout);

    // Set up a timer to call the state machine
    timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, []() {
        state_machine(); // Call the state machine function
    });
    timer->start(33);   // ms per UI frame

    // Show the window
    window.show(); // Show the application window (not full screen)

    // Execute the application
    int result = app.exec();

    // Safety commands before cleanup
    onLightOffButtonClicked();      // Turn off the light
    onFanOffButtonClicked();        // Turn off the fan

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100 milliseconds

    // Clean up Marlin controller
    delete marlin;

    // Stop the camera when the application exits
    cam.stopVideo(); // Stop the video mode if it's running
    cam.stopPhoto();

    return result;
}