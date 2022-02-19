
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

using namespace std;
using namespace frc;
using namespace VisionConstants;

class VisionSubsystem : public frc2::SubsystemBase
{
public:
    VisionSubsystem(Team1259::Gyro *gyro, TurretSubsystem *turret);

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

    units::meter_t calcResidual(units::meter_t radius, vector<frc::Translation2d> points, frc::Translation2d center);

    bool FitCircle(vector<frc::Translation2d> targetVectors, units::meter_t precision, int maxAttempts);

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

    double m_tx;
    double m_ty;
    int m_consecNoTargets;
    bool m_validTarget;
    double m_smoothedRange;
    frc::Pose2d m_robotPose;
    vector<double> m_averageDistance;
    vector<double> m_averageAngle;
    photonlib::PhotonCamera camera{"gloworm"};
    //std::vector<std::pair<double, double>> m_centerPoints;
    frc::Translation2d m_cameraToHub;
    units::inch_t kCameraHeight;
    units::inch_t kCurrTargetHeight;
    units::degree_t kCameraPitch;
    units::degree_t kTargetPitch;

    /// Gyro to determine field relative angles, from @ref RobotContainer
    Team1259::Gyro *m_gyro;
    TurretSubsystem *m_turret;
};
