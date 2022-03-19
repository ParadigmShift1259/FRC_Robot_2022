
#pragma once

#include "Constants.h"
#include "../IOdometry.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/EntryListenerFlags.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonTrackedTarget.h>

#include <vector>

#include <wpi/numbers>
#include <wpi/span.h>

#include "Constants.h"
#include "common/Util.h"

#include "Gyro.h"
#include "TurretSubsystem.h"
#include "HoodSubsystem.h"
#include "common/DebugFlag.h"

using namespace std;
using namespace frc;
using namespace units;
using namespace VisionConstants;

const frc::Translation2d kHubCenter = frc::Translation2d(kFieldLength/2, kFieldWidth/2);  // TO DO make a constant
const meter_t kHubRadius = foot_t(2.0);
const frc::Translation2d turretCenterToRobotCenter = frc::Translation2d(3_in, 0_in);   // TO DO make a constant


class VisionSubsystem : public frc2::SubsystemBase
{
public:
    VisionSubsystem(Team1259::Gyro *gyro, TurretSubsystem& turret, HoodSubsystem& hood, IOdometry& odometry);

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

    meter_t calcResidual(meter_t radius, vector<frc::Translation2d> points, frc::Translation2d center);

    frc::Translation2d FitCircle(vector<frc::Translation2d> targetVectors, meter_t precision, int maxAttempts);

    double GetHubAngle();

    double GetHubDistance(bool smoothed);

    units::radian_t GetVectorAngle(Translation2d vector);

    void CloseLogFile() { if (m_logFile) fclose(m_logFile);  }

protected:
    /// Converts degrees to radians
    /// \param degrees Degrees to convert
    double DegreesToRadians(double degrees);
    void Work();

private:    
    void GetVisionTargetCoords(wpi::span<const photonlib::PhotonTrackedTarget>& targets, vector<frc::Translation2d>& targetVectors);   // TODO targetCoords
    frc::Translation2d FindAverageOfTargets(vector<frc::Translation2d>& targetVectors);
    // TODO make it FindMedianOfTargets
    void FilterTargets(vector<frc::Translation2d>& targetVectors, frc::Translation2d center, meter_t rMax, degree_t minangle, degree_t maxangle);
    void GetFieldReleativeRobotAndCameraPoses(frc::Translation2d& cameraToHub, photonlib::PhotonPipelineResult& result, Rotation2d& fieldToCamRot, Translation2d& camToRobotCenter);
    Translation2d CompensateMotionForLatency(Rotation2d& fieldToCamRot, Translation2d& camToRobotCenter);
    Translation2d Targeting();
    void SteerTurretAndAdjusthood();

    shared_ptr<nt::NetworkTable> m_networktable;
    bool m_led;

    int m_consecNoTargets;
    bool m_validTarget;
    double m_smoothedRange;
    Pose2d m_robotPose;
    photonlib::PhotonCamera camera{"gloworm"};
    frc::Translation2d m_cameraToHub;
    Pose2d m_robotvisionPose;
    Pose2d m_cameraPose;
    units::time::second_t m_visionTimestamp;

    /// Gyro to determine field relative angles, from @ref RobotContainer
    Team1259::Gyro *m_gyro;
    TurretSubsystem& m_turret;
    HoodSubsystem& m_hood;
    IOdometry& m_odometry;
    FILE* m_logFile = nullptr;

    DebugFlag   m_dbgLogInvalid{"VisLogInvalid", true};
    DebugFlag   m_dbgLogTargetData{"VisLogTargetData", true};
    DebugFlag   m_dbgUseUseVisionForTurret{"UseVisionForTurret", true};
};
