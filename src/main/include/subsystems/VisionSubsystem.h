
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/EntryListenerFlags.h>
#include <photonlib/PhotonCamera.h>
#include <vector>

#include <wpi/numbers>

#include "Constants.h"
#include "common/Util.h"

#include "Gyro.h"
#include "TurretSubsystem.h"
#include "common/DebugFlag.h"

using namespace std;
using namespace frc;
using namespace units;
using namespace VisionConstants;

class VisionSubsystem : public frc2::SubsystemBase
{
public:
    VisionSubsystem(Gyro2 *gyro, TurretSubsystem *turret);

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    void NTcallback(nt::NetworkTable* table, std::string_view name, nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags);

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

    meter_t calcResidual(meter_t radius, vector<Translation2d> points, Translation2d center);

    bool FitCircle(vector<Translation2d> targetVectors, meter_t precision, int maxAttempts);

    double GetHubAngle();

    double GetHubDistance(bool smoothed);

protected:
    /// Converts degrees to radians
    /// \param degrees Degrees to convert
    double DegreesToRadians(double degrees);

private:    
    shared_ptr<nt::NetworkTable> m_dashboard;
    shared_ptr<nt::NetworkTable> m_networktable;
    bool m_led;

    int m_consecNoTargets;
    bool m_validTarget;
    double m_smoothedRange;
    Pose2d m_robotPose;
    vector<double> m_averageDistance;
    vector<double> m_averageAngle;
    photonlib::PhotonCamera camera{"gloworm"};
    //std::vector<std::pair<double, double>> m_centerPoints;
    Translation2d m_cameraToHub;
    inch_t kCameraHeight;
    inch_t kCurrTargetHeight;
    degree_t kCameraPitch;
    degree_t kTargetPitch;

    /// Gyro to determine field relative angles, from @ref RobotContainer
    Gyro2 *m_gyro;
    TurretSubsystem *m_turret;

    DebugFlag   m_dbgLogInvalid{"VisLogInvalid", false};
    DebugFlag   m_dbgLogTargetData{"VisLogTargetData", false};
    DebugFlag   m_dbgUseUseVisionForTurret{"UseVisionForTurret", true};
};
