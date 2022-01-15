
#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <wpi/numbers>

#include "Constants.h"
#include "common/Util.h"

using namespace std;
using namespace frc;
using namespace VisionConstants;

class VisionSubsystem : public frc2::SubsystemBase
{
public:
    VisionSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Determine valid vision based on returned distance values
    /// \return         Whether or not the Vision Subsystem is giving accurate values
    bool GetValidTarget();
    /// Retrieves the distance calculation from the target via the limelight
    double GetDistance();
    /// \return         The angle calculation from the target via the limelight
    double GetAngle();
    /// Turns the limelight LED on or off
    /// \param on        Boolean where true = LED on
    void SetLED(bool on);

protected:
    /// Converts degrees to radians
    /// \param degrees Degrees to convert
    double DegreesToRadians(double degrees);

private:    
    shared_ptr<nt::NetworkTable> m_dashboard;
    shared_ptr<nt::NetworkTable> m_networktable;
    bool m_led;

    double m_tx;
    double m_ty;
    bool m_validTarget;
    vector<double> m_averageDistance;
    vector<double> m_averageAngle;
};
