#pragma once

#include <wpi/numbers>
#include "Constants.h"

using namespace std;


class Util
{
public:
    /// Convert any angle theta in degrees to radians
    static double DegreesToRadians(double theta);

    /// Convert any angle theta in radians to degrees
    static double RadiansToDegrees(double theta);

    /// Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
    /// \param theta    any angle in radians
    /// \return         any angle within the interval [0, 2pi]
    static double ZeroTo2PiRads(double theta);

    /// Convert any angle theta in degrees to its equivalent on the interval [0, 360]
    /// \param theta    any angle in degrees
    /// \return         any angle within the interval [0, 360]
    static double ZeroTo360Degs(double theta);

    /// Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
    /// \param theta    any angle in radians
    /// \return         any angle within the interval [-pi, pi]
    static double NegPiToPiRads(double theta);

    /// Get the average of a double vector
    /// \param numbers  vector of doubles
    /// \return         average of doubles in vector
    static double GetAverage(vector<double> numbers);

    /// If an inputValue is smaller than its deadzone, returns 0, otherwise returns the inputValue
    static double Deadzone(double inputValue, double deadzone)
    {
        // If the input is small return 0
        return abs(inputValue) <= deadzone ? 0 : inputValue;
    }   
};
