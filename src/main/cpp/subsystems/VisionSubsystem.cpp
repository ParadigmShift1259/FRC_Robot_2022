
#include "subsystems/VisionSubsystem.h"
#include <fmt/core.h>
#include <vector>
#include <photonlib/PhotonUtils.h>

VisionSubsystem::VisionSubsystem(Team1259::Gyro *gyro, TurretSubsystem& turret, HoodSubsystem& hood, IOdometry& odometry) 
 : m_networktable(nt::NetworkTableInstance::GetDefault().GetTable("photonvision"))
 , m_led(true)
 , m_validTarget(false)
 , m_gyro(gyro)
 , m_turret(turret)
 , m_hood(hood)
 , m_odometry(odometry)
{
    SetLED(true);
    m_consecNoTargets = 0;

    m_targeting = kOdometry; // kPureVision;
    m_enableVisionOdoCorrection = false;

    wpi::log::DataLog& log = DataLogManager::GetLog();

    m_logNumRawTargets = wpi::log::IntegerLogEntry(log, "/vision/numRawTargets");
    m_logNumFilteredTargets = wpi::log::IntegerLogEntry(log, "/vision/numFilteredTargets");
    m_logRobotvisionPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotPoseX");
    m_logRobotvisionPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotPoseY");
    m_logRobotvisionPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotPoseTheta");
    m_logCameraPoseX = wpi::log::DoubleLogEntry(log, "/vision/cameraPoseX");
    m_logCameraPoseY = wpi::log::DoubleLogEntry(log, "/vision/cameraPoseY");
    m_logCameraPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/cameraPoseTheta");
    m_logCameraToHubDist = wpi::log::DoubleLogEntry(log, "/vision/cameraToHubDist");
    m_logCameraToHubAngle = wpi::log::DoubleLogEntry(log, "/vision/cameraToHubAngle");
    m_logCircleFitResultDist = wpi::log::DoubleLogEntry(log, "/vision/circleFitDist");
    m_logCircleFitResultAngle = wpi::log::DoubleLogEntry(log, "/vision/circleFitAngle");

    // m_networktable->AddEntryListener(
    //    "gloworm/latencyMillis"
    //    ,[this](nt::NetworkTable* table
    //         , std::string_view name
    //         , nt::NetworkTableEntry entry
    //         , std::shared_ptr<nt::Value> value
    //         , int flags)
    //     { NTcallback(table, name, entry, value, flags); }
    //     , nt::EntryListenerFlags::kUpdate);
}

void VisionSubsystem::NTcallback(nt::NetworkTable* table, std::string_view name, nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags)
{
    // static units::time::second_t ntcallbackTimestamp = Timer::GetFPGATimestamp();
    // SmartDashboard::PutNumber("Dt", (Timer::GetFPGATimestamp() - ntcallbackTimestamp).to<double>());
    // ntcallbackTimestamp = Timer::GetFPGATimestamp();

    // // printf("Dt: %f\n", (Timer::GetFPGATimestamp() - visionTimestamp).to<double>());
    // Work(ntcallbackTimestamp);

    // m_logRobotvisionPoseX.Append(m_robotvisionPose.X().to<double>());
    // m_logRobotvisionPoseY.Append(m_robotvisionPose.Y().to<double>());
    // m_logRobotvisionPoseTheta.Append(m_robotvisionPose.Rotation().Degrees().to<double>());
    // m_logCameraPoseX.Append(m_cameraPose.X().to<double>());
    // m_logCameraPoseY.Append(m_cameraPose.Y().to<double>());
    // m_logCameraPoseTheta.Append(m_cameraPose.Rotation().Degrees().to<double>());
    // m_logCameraToHubDist.Append(m_cameraToHub.Norm().to<double>());
    // m_logCameraToHubAngle.Append(GetVectorAngle(m_cameraToHub).to<double>()*180/3.14159);
}

void VisionSubsystem::Periodic()
{
    Work(Timer::GetFPGATimestamp());

    m_logRobotvisionPoseX.Append(m_robotvisionPose.X().to<double>());
    m_logRobotvisionPoseY.Append(m_robotvisionPose.Y().to<double>());
    m_logRobotvisionPoseTheta.Append(m_robotvisionPose.Rotation().Degrees().to<double>());
    m_logCameraPoseX.Append(m_cameraPose.X().to<double>());
    m_logCameraPoseY.Append(m_cameraPose.Y().to<double>());
    m_logCameraPoseTheta.Append(m_cameraPose.Rotation().Degrees().to<double>());
    m_logCameraToHubDist.Append(m_cameraToHub.Norm().to<double>());
    m_logCameraToHubAngle.Append(GetVectorAngle(m_cameraToHub).to<double>()*180/3.14159);
}
    
void VisionSubsystem::Work(units::time::second_t timestamp)
{
    second_t visionTimestamp = Timer::GetFPGATimestamp();

    bool bLogInvalid = m_dbgLogInvalid;

    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    bool validTarget = result.HasTargets();
    // printf("valid target %d\n", validTarget);
    if (validTarget)
    {
        vector<frc::Translation2d> targetVectors;
        auto targets = result.GetTargets();
        GetVisionTargetCoords(targets, targetVectors);

        m_logNumRawTargets.Append(targetVectors.size());

        //printf("pitch-filtered targets: %d   ", targetVectors.size());

        // if (m_targeting == kOdometry && m_odometry.OdoValid())
        // {
        //     FilterTargets(targetVectors, m_cameraToHub, kMaxTargetRadialSpreadOdo, kMaxTargetAngleSpreadOdo); 
        // }
        // else
        // {
        //     // TO DO: use median instead of average and adjust radius
        //     frc::Translation2d averageTarget = FindAverageOfTargets(targetVectors);
        //     FilterTargets(targetVectors, averageTarget, kMaxTargetRadialSpreadPureVision, kMaxTargetAngleSpreadPureVision);
        // }

        m_logNumFilteredTargets.Append(targetVectors.size());

        //printf("outlier-filtered targets: %d   ", targetVectors.size());
        if (targetVectors.size() >= 3 && targetVectors.size() <= 6)
        {
            frc::Translation2d cameraToHub = FitCircle(targetVectors, meter_t{0.01}, 20);
            if (cameraToHub != frc::Translation2d())  // FitCircle returns (0,0) translation to indicate failue
            {
                // cameraToHub is the vector from cam to hub IN CAMERA-RELATIVE COORDINATE SYSTEM!
                // printf("camera pose from circle fit: x %.3f y %.3f    ", m_cameraToHub.X().to<double>(), m_cameraToHub.Y().to<double>());
                m_logCircleFitResultDist.Append(cameraToHub.Norm().to<double>());
                m_logCircleFitResultAngle.Append(GetVectorAngle(cameraToHub).to<double>()*180/3.14159);
                m_consecNoTargets = 0;
                m_validTarget = true;
                m_visionTimestamp = timestamp - result.GetLatency();

                UpdateFieldReleativeRobotAndCameraPoses(cameraToHub);

                if(m_enableVisionOdoCorrection)
                {
                    if(m_odometry.GetState(m_visionTimestamp).velocity < meters_per_second_t{.1})
                        {
                        m_odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>(0.01, 0.01, 0.01));
                        m_odometry.AddVisionMeasurement(m_robotvisionPose, m_visionTimestamp);
                        }
                    else
                        {
                        m_odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3>(0.05, 0.05, 0.01));
                        m_odometry.AddVisionMeasurement(m_robotvisionPose, m_visionTimestamp);
                        }
                }
                m_cameraToHub = CompensateForLatencyAndMotion();    // Use wheel odo to correct for movement since image was captured

                // do Hub distance smoothing
                if (m_smoothedRange > 0)
                    m_smoothedRange = kRangeSmoothing * m_smoothedRange + (1 - kRangeSmoothing) * GetHubDistance(false);
                else
                    m_smoothedRange = GetHubDistance(false);
            }
            else // FitCircle() failed
            {
                frc::DataLogManager::Log("Circle fit failed");
                if (bLogInvalid)
                    printf("Circle fit failed \n");
                validTarget =  false;
            }
        }
        else
        {
            frc::DataLogManager::Log(fmt::format("Only {} vision targets", targetVectors.size()));
            if (bLogInvalid)
                printf("Only %d  vision targets\n", targetVectors.size());
            validTarget =  false; 
        }
    }
    else
    { 
        m_consecNoTargets++;
        if (m_consecNoTargets >= kVisionFailLimit)
        {
            m_validTarget = false;
            m_smoothedRange = 0;
        }
    }

    if (!m_odometry.OdoValid())
    {   
        // This assumes gyro was properly zeroed previously and is still valid, otherwise odometry coordinates will have arbitrary rotation w.r.t. true field!
        if (m_dbgLogTargetData)
        {
        printf("Odometry Invalid: x=%.3f, y=%.3f, heading =%.1f\n", m_odometry.GetPose().X().to<double>(), m_odometry.GetPose().Y().to<double>(), m_odometry.GetPose().Rotation().Degrees().to<double>());
        }
        m_odometry.ResetOdometry(m_robotvisionPose);
        if (m_dbgLogTargetData)
        {
            printf("Resetting Odometry from Vision: x=%.3f, y=%.3f, heading =%.1f\n", m_odometry.GetPose().X().to<double>(), m_odometry.GetPose().Y().to<double>(), m_odometry.GetPose().Rotation().Degrees().to<double>());
        }
        frc::DataLogManager::Log(fmt::format("Resetting Odometry from Vision: x={}, y={}, heading={}", m_odometry.GetPose().X().to<double>(), m_odometry.GetPose().Y().to<double>(), m_odometry.GetPose().Rotation().Degrees().to<double>()));
    }

    SmartDashboard::PutNumber("VisionDistance: ", GetHubDistance(false) * 39.37);

    if (m_targeting == TargetingMode::kOdometry && m_odometry.OdoValid())
        {
        m_cameraToHub = Targeting();
        m_validTarget = true;
        }

    if (m_targeting != TargetingMode::kOff)
        SteerTurretAndAdjusthood();

    // diagnostic printouts...
    static int counter=0;
    if (counter++ % 25 == 0 && m_dbgLogTargetData)
    {
        printf("Odometry Pose: x=%.3f, y=%.3f, theta=%.1f\n", m_odometry.GetPose().X().to<double>()* 39.37, m_odometry.GetPose().Y().to<double>()* 39.37, m_odometry.GetPose().Rotation().Degrees().to<double>());
        if (validTarget)
            {
            printf("Vision Pose..: x=%.3f, y=%.3f, theta=%.1f\n", m_robotvisionPose.X().to<double>()* 39.37, m_robotvisionPose.Y().to<double>()* 39.37, m_robotvisionPose.Rotation().Degrees().to<double>());
//            printf("camera pose..: x=%.3f, y=%.3f, theta %.3f\n", m_cameraPose.X().to<double>()* 39.37, m_cameraPose.Y().to<double>()* 39.37, m_cameraPose.Rotation().Degrees().to<double>());
            }  
        else
            printf("NO Vision Pose\n");
        printf(".\n");

        // if (!m_validTarget && bLogInvalid)
        // {
        //     //printf("PhotonCam Has No Targets!\n");
        // }
        // else if (m_dbgLogTargetData)
        // {
        //     //printf("Angle: %f, Range: %f, Robot X %f, Y: %f, Theta: %f\n", GetHubAngle() *180/3.14, GetHubDistance(true) * 39.37, m_robotPose.X().to<double>() * 39.37,m_robotPose.Y().to<double>() * 39.37,m_robotPose.Rotation().Degrees().to<double>()); 
        //     // std::cout << "Center: (" << (double)m_cameraToHub.X() << "," << (double)m_cameraToHub.Y() << "). ";
        //     // std::cout << "Angle:  " << GetHubAngle() *180/3.14<< ", ";
        //     // std::cout << "Range: " << GetHubDistance(true) * 39.37 << ", ";
        //     // std::cout << "Robot X: " << (double) m_robotPose.X() * 39.37 << ", Y: " << (double) m_robotPose.Y() * 39.37 << ", Theta: ", m_robotPose.Rotation().Degrees().to<double>();
        //     // for(int i = 0; i < targetVectors.size(); i++) {
        //     //     std::cout << "(" << (double)targetVectors[i].X() << "," << (double)targetVectors[i].Y() << "). ";
        //     // }
        //     //std::cout << std::endl;
            
        // }
    }
 
    SmartDashboard::PutNumber("D_V_Active", m_validTarget);
    // SmartDashboard::PutNumber("D_V_Distance", distance);
    // SmartDashboard::PutNumber("D_V_Angle", m_horizontalangle);
    SmartDashboard::PutNumber("Wk", (Timer::GetFPGATimestamp() - m_visionTimestamp).to<double>());
}

void VisionSubsystem::GetVisionTargetCoords(wpi::span<const photonlib::PhotonTrackedTarget>& targets, vector<frc::Translation2d>& targetVectors)
{
    // Gets camera-relative x,y translations for each vision target
    for (size_t i = 0; i < targets.size(); i++)
    {
        degree_t TargetPitch = degree_t{targets[i].GetPitch()};
        meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
            kCameraHeight, kCurrTargetHeight, kCameraPitch, TargetPitch);
        if ((TargetPitch > units::degree_t{-13}) && (TargetPitch < units::degree_t{24}))
            targetVectors.push_back(photonlib::PhotonUtils::EstimateCameraToTargetTranslation(range, frc::Rotation2d(degree_t{-targets[i].GetYaw()})));
        else
            {
            // printf("discarded pitch = %f \n", TargetPitch.to<double>());
            frc::DataLogManager::Log(fmt::format("Discarded target: pitch={}", TargetPitch.to<double>()));
            }
    }
}

frc::Translation2d  VisionSubsystem::FindAverageOfTargets(vector<frc::Translation2d>& targetVectors)// TODO make it FindMedianOfTargets
{
    double xTotal = 0;
    double yTotal = 0;
    for (size_t i = 0; i < targetVectors.size(); i++)
    {
        xTotal += (double)targetVectors[i].X();
        yTotal += (double)targetVectors[i].Y();
    }
    double xMean = xTotal/targetVectors.size();
    double yMean = yTotal/targetVectors.size();

    return  Translation2d(meter_t{xMean}, meter_t{yMean});
}

void VisionSubsystem::FilterTargets(vector<frc::Translation2d>& targetVectors, frc::Translation2d center, meter_t rMax, degree_t angleTol)
{
    for (size_t i = 0; i < targetVectors.size(); i++)
    {
        Translation2d r = targetVectors[i] - center;

        if (r.Norm() > rMax || 
            (GetVectorAngle(r) < units::radian_t{GetVectorAngle(center) + angleTol} && GetVectorAngle(r) > units::radian_t{GetVectorAngle(center) - angleTol}))
        {
            targetVectors.erase(targetVectors.begin() + i);
            i--;
            frc::DataLogManager::Log(fmt::format("Discarded target: r={}, angle={}", r.Norm().to<double>()*39.37, GetVectorAngle(r).to<double>()*180/3.14));
        }
    }
}

void  VisionSubsystem::UpdateFieldReleativeRobotAndCameraPoses(frc::Translation2d& cameraToHub)
{
    StateHist delayedState = m_odometry.GetState(m_visionTimestamp);
    frc::Pose2d delayedOdoPose = delayedState.pose;
    degree_t angleTurret = delayedState.m_turretAngle;
    Rotation2d robotRot = delayedOdoPose.Rotation(); // robot heading FIELD RELATIVE
    Rotation2d fieldToCamRot = robotRot + Rotation2d(angleTurret + 180_deg);  

    Translation2d camToTurretCenterRRC = Translation2d(-5_in, 0_in).RotateBy(Rotation2d{angleTurret});  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenterRRC = camToTurretCenterRRC + turretCenterToRobotCenter;  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenter = camToRobotCenterRRC.RotateBy(robotRot);  // FIELD RELATIVE COORDINATES

    // cameraToHub is the vector from cam to hub IN CAMERA-RELATIVE COORDINATE SYSTEM. Transform to Field Relative...
    m_cameraPose = Pose2d(kHubCenter - cameraToHub.RotateBy(fieldToCamRot), fieldToCamRot); // FIELD RELATIVE cam pose
    
    m_robotvisionPose = Pose2d(m_cameraPose.Translation() + camToRobotCenter, robotRot);  // FIELD RELATIVE robot pose
}

Translation2d  VisionSubsystem::CompensateForLatencyAndMotion()
{
    StateHist currentState = m_odometry.GetState();
    StateHist delayedState = m_odometry.GetState(m_visionTimestamp);
 
    Transform2d compenstaion = Transform2d(delayedState.pose, currentState.pose);

    Pose2d compensatedRobotvisionPose = m_robotvisionPose.TransformBy(compenstaion);

    // static int ctr=0;
    // if (compenstaion.Translation().Norm() > .01_m && ctr++ % 10 == 0)
    //     {
    //     // Transform2d compenstaion; // zero transform for testing
    //     printf("current pose: x %.3f y %.3f\n", currentState.pose.X().to<double>(), currentState.pose.Y().to<double>());
    //     printf("delayed pose: x %.3f y %.3f\n", delayedState.pose.X().to<double>(), delayedState.pose.Y().to<double>());
    //     printf("compenstaion: x %.3f y %.3f\n", compenstaion.X().to<double>(), compenstaion.Y().to<double>());
    //     printf("vision  pose: x %.3f y %.3f\n", m_robotvisionPose.X().to<double>(), m_robotvisionPose.Y().to<double>());
    //     printf("comp'd Vpose: x %.3f y %.3f\n", compensatedRobotvisionPose.X().to<double>(), compensatedRobotvisionPose.Y().to<double>());
    //     printf(".\n");
    //     }

    degree_t angleTurret = currentState.m_turretAngle;
    Rotation2d robotRot = currentState.pose.Rotation(); // robot heading FIELD RELATIVE
    Rotation2d fieldToCamRot = robotRot + Rotation2d(angleTurret + 180_deg);  

    Translation2d camToTurretCenterRRC = Translation2d(-5_in, 0_in).RotateBy(Rotation2d{angleTurret});  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenterRRC = camToTurretCenterRRC + turretCenterToRobotCenter;  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenter = camToRobotCenterRRC.RotateBy(robotRot);  // FIELD RELATIVE COORDINATES

    Pose2d compensatedCameraPose = Pose2d(compensatedRobotvisionPose.Translation() - camToRobotCenter, fieldToCamRot);  // FIELD RELATIVE COORDINATES    
    Translation2d cameraToHubFR = kHubCenter - compensatedCameraPose.Translation(); // FIELD RELATIVE COORDINATES    

    return cameraToHubFR.RotateBy(-fieldToCamRot); // transform from field-relative back to cam-relative
}

Translation2d  VisionSubsystem::Targeting()
{
    // use odometry instead of vision
    StateHist lastOdoState = m_odometry.GetState();
    degree_t angleTurret = lastOdoState.m_turretAngle;

    Pose2d robotPose = lastOdoState.pose;
    // frc::Translation2d camToTurretCenter = frc::Translation2d(meter_t{(cos(angleTurret) * inch_t{-12})}, meter_t{(sin(angleTurret) * inch_t{-12})});
    // frc::Transform2d camreaTransform = frc::Transform2d(camToTurretCenter + turretCenterToRobotCenter, radian_t{angleTurret});
    // frc::Rotation2d fieldToCamAngle = m_robotPose.Rotation() + frc::Rotation2d(units::radian_t{angleTurret});  
    // m_cameraToHub = kHubCenter - m_robotPose.TransformBy(camreaTransform.Inverse()).Translation();

    Rotation2d robotRot = robotPose.Rotation(); // robot heading FIELD RELATIVE
    Rotation2d fieldToCamRot = robotRot + Rotation2d(angleTurret + 180_deg);  
    Translation2d camToTurretCenterRRC = Translation2d(-5_in, 0_in).RotateBy(Rotation2d{angleTurret});  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenterRRC = camToTurretCenterRRC + turretCenterToRobotCenter;  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenter = camToRobotCenterRRC.RotateBy(robotRot);  // FIELD RELATIVE COORDINATES                
    Pose2d cameraPose = Pose2d(robotPose.Translation() - camToRobotCenter, fieldToCamRot);  // FIELD RELATIVE COORDINATES    
    Translation2d cameraToHubFR = kHubCenter - cameraPose.Translation(); // FIELD RELATIVE COORDINATES    

    return cameraToHubFR.RotateBy(-fieldToCamRot); // transform from field-relative back to cam-relative
}

void  VisionSubsystem::SteerTurretAndAdjusthood()
{
    static int turretCmdHoldoff = 0;
    double turretCmdP;

    if (turretCmdHoldoff > 0)
        turretCmdHoldoff--;
    
    if(m_targeting == kOdometry || (m_targeting == kPureVision && turretCmdHoldoff == 0))
    {
        if (m_cameraToHub.Norm() > 0_m && m_cameraToHub.Norm() < kMaxTargetRange)
            {
            if (m_targeting == kPureVision)
                {
                turretCmdHoldoff = kTurretCmdHoldoff;
                turretCmdP = kTurretCmdP;
                }
            else    
                {
                turretCmdHoldoff = 0;
                turretCmdP = 1;
                }

            auto hubAngle = GetHubAngle() * 180.0 / wpi::numbers::pi;
            // Aim off by 3 degrees to get a switl shot? hubAngle += 3.0;
            m_turret.TurnToRelative(hubAngle * kTurretCmdP); // can apply P constant < 1.0 if needed for vision tracking stability 
            m_hood.SetByDistance(GetHubDistance(false));
            }
    }
            
    if (m_dbgLogTargetData)
    {
        // printf("Turret Angle %.2f   ", m_turret.GetCurrentAngle());
        // printf("Hub Angle: %.2f \n", hubAngle);
        // printf( " Hub angle: %f  range: %f\n", GetHubAngle()*180/3.14159, GetHubDistance(true)*39.37);
    }
}

bool VisionSubsystem::GetValidTarget()
{
    return m_validTarget;
}

void VisionSubsystem::SetLED(bool on)
{
    m_led = on;
    camera.SetLEDMode(m_led ? photonlib::LEDMode::kDefault : photonlib::LEDMode::kOff);
}


frc::Translation2d VisionSubsystem::FitCircle(vector<frc::Translation2d> targetVectors, meter_t precision, int maxAttempts)
{
    double xSum = 0.0;
    double ySum = 0.0;
    for (size_t i = 0; i < targetVectors.size(); i++) 
    {
        xSum += (double) targetVectors[i].X();
        ySum += (double) targetVectors[i].Y();
    }
    frc::Translation2d cameraToHub = frc::Translation2d(meter_t{xSum / targetVectors.size()} + kVisionTargetRadius, meter_t{ySum / targetVectors.size()});

    // Iterate to find optimal center
    meter_t shiftDist = kVisionTargetRadius / 2.0;
    meter_t minResidual = calcResidual(kVisionTargetRadius, targetVectors, cameraToHub);

    int n = 0;

    while (n < maxAttempts) 
    {
        frc::Translation2d translation = Translation2d(shiftDist, meter_t{0.0});
        frc::Translation2d bestPoint = cameraToHub;
        bool centerIsBest = true;

        // Check all adjacent positions
        for (int i = 0; i < 4; i++) 
        {
            meter_t residual =
                calcResidual(kVisionTargetRadius, targetVectors, cameraToHub + translation);
            if (residual < minResidual) {
                bestPoint = cameraToHub + (translation);
                minResidual = residual;
                centerIsBest = false;
                break;
            }
            translation = translation.RotateBy(frc::Rotation2d(i * degree_t{90}));
        }
        // Decrease shift, exit, or continue
        if (centerIsBest) {
            shiftDist /= 2.0;
            if (shiftDist < precision) {
                return cameraToHub;
            }
        } else {
            cameraToHub = bestPoint;
        }

        n++;
    }
    // failed - returns 0 translation
    return Translation2d();
}

 meter_t VisionSubsystem::calcResidual(meter_t radius, vector<frc::Translation2d> points, frc::Translation2d center)
{
    double residual = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
      double diff = (double) (points[i].Distance(center) - radius);
      residual += diff * diff;
    }
    return meter_t{residual};
}

double VisionSubsystem::GetHubAngle()
{
    return (double) GetVectorAngle(m_cameraToHub);
}

units::radian_t VisionSubsystem::GetVectorAngle(Translation2d vector)
{
    return units::radian_t{atan2((double)vector.Y(), (double)vector.X())};
}

double VisionSubsystem::GetHubDistance(bool smoothed)
{
    if (smoothed && m_smoothedRange > 0)
        return m_smoothedRange;

    return (double) m_cameraToHub.Norm();
}


void VisionSubsystem::SetTargetingMode(VisionSubsystem::TargetingMode mode)
{
    m_targeting = mode;
}

VisionSubsystem::TargetingMode VisionSubsystem::GetTargetingMode(void)
{
    return m_targeting;
}

void VisionSubsystem::ToggleTargetingMode(void)
{
if (GetTargetingMode() == VisionSubsystem::TargetingMode::kOdometry)
    SetTargetingMode(VisionSubsystem::TargetingMode::kPureVision);
else if (GetTargetingMode() == VisionSubsystem::TargetingMode::kPureVision)
    SetTargetingMode(VisionSubsystem::TargetingMode::kOdometry);
SmartDashboard::PutBoolean("Using Odometry ", GetTargetingMode() == VisionSubsystem::TargetingMode::kOdometry);
}

void VisionSubsystem::EnableOdoCorrection()
{
m_enableVisionOdoCorrection = true;
}

void VisionSubsystem::DisableOdoCorrection()
{
m_enableVisionOdoCorrection = false;
}

void VisionSubsystem::CamCapture(void)
{
camera.TakeInputSnapshot();
camera.TakeOutputSnapshot();
}
