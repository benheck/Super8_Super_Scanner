#include "MarlinController.h"
#include <iostream>
#include <thread>
#include <chrono>

MarlinController::MarlinController(const std::string &port, unsigned int baud_rate)
    : serial(io, port) {
    try {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        std::cout << "Connected to Marlin on port " << port << " with baud rate " << baud_rate << std::endl;

        // Set Marlin to relative positioning mode
        sendAndWait("G91"); // G91 sets relative positioning mode
        std::cout << "Marlin set to relative positioning mode (G91)." << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error initializing MarlinController: " << e.what() << std::endl;
        throw;
    }
}

MarlinController::~MarlinController() {
    if (serial.is_open()) {
        serial.close();
    }
}

void MarlinController::sendGCode(const std::string &gcode) {
    try {
        std::string command = gcode + "\n";
        boost::asio::write(serial, boost::asio::buffer(command));
        std::cout << "Sent: " << gcode << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error sending G-code: " << e.what() << std::endl;
    }
}

std::string MarlinController::readResponse() {
    try {
        boost::asio::streambuf responseBuffer;
        boost::asio::read_until(serial, responseBuffer, "\n");
        std::istream responseStream(&responseBuffer);
        std::string response;
        std::getline(responseStream, response);
        std::cout << "Received: " << response << std::endl;
        return response;
    } catch (const std::exception &e) {
        std::cerr << "Error reading response: " << e.what() << std::endl;
        return "";
    }
}

void MarlinController::sendAndWait(const std::string &gcode) {
    sendGCode(gcode);
    while (!checkForOK()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool MarlinController::checkForOK() {
    std::string response = readResponse();
    if (response.find("ok") != std::string::npos) {
        if (eatOKs > 0) {
            --eatOKs;
        }
        return eatOKs == 0;
    }
    return false;
}

void MarlinController::waitForMoveCompletion() {
    sendAndWait("M400"); // M400 waits for all movements to complete
}

void MarlinController::stopAllMotion() {
    sendAndWait("M112"); // M112 is the emergency stop command
}

void MarlinController::advanceFilm(float mm, float speed, bool wait) {
    // Generate G-code for moving the Y-axis
    std::string gcode = "G1 Y" + std::to_string(mm) + " F" + std::to_string(speed);

    if (wait) {
        sendAndWait(gcode); // Send the G-code and wait for OK
        waitForMoveCompletion(); // Ensure all movements are complete
    } else {
        sendGCode(gcode); // Send the G-code without waiting
    }
}

void MarlinController::homeAxes() {
    sendAndWait("G28"); // G28 homes all axes
}

void MarlinController::zeroAxes() {
    sendAndWait("G92 X0 Y0 Z0"); // G92 sets the current position to zero
}

void MarlinController::enableMotors() {
    sendAndWait("M17"); // M17 enables all motors
}

void MarlinController::disableMotors() {
    sendAndWait("M18"); // M18 disables all motors
}

void MarlinController::enableSingleMotor(const std::string &axis) {
    sendAndWait("M17 " + axis);
}

void MarlinController::disableSingleMotor(const std::string &axis) {
    sendAndWait("M18 " + axis); // M18 disables the specified motor
}

