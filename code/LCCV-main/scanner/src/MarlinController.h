#ifndef MARLIN_CONTROLLER_H
#define MARLIN_CONTROLLER_H

#include <boost/asio.hpp>
#include <string>

class MarlinController {
private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;

    int eatOKs = 0; // Number of OKs to eat

public:
    MarlinController(const std::string &port, unsigned int baud_rate);
    ~MarlinController();

    bool checkForOK();

    void sendGCode(const std::string &gcode);
    std::string readResponse();
    void sendAndWait(const std::string &gcode);

    void waitForMoveCompletion();
    void stopAllMotion();

    void advanceFilm(float mm, float speed = 100.0f, bool wait = false); // Move the Y-axis by a specified distance in mm
    void advanceFilm2(float mmY, float unitsX, float speed = 100.0f, bool wait = false); // Move the Y-axis by a specified distance in mm
    void homeAxes();            // Home all axes (X, Y, Z)
    void zeroAxes();            // Zero out the X and Y axes

    void enableMotors();        // Enable all motors
    void disableMotors();       // Disable all motors
    void enableSingleMotor(const std::string &axis); // Enable a specific motor (e.g., X, Y, Z)
    void disableSingleMotor(const std::string &axis); // Disable a specific motor (e.g., X, Y, Z)
};

#endif // MARLIN_CONTROLLER_H