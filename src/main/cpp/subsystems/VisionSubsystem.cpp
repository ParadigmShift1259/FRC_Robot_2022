
#include "subsystems/VisionSubsystem.h"
#include <units/math.h>
//#include <iostream>
#include <fmt/core.h>
#include <photonlib/PhotonUtils.h>
#include <vector>

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

    m_logFile = stderr; // fopen("/tmp/visionLog.txt", "w");

   m_networktable->AddEntryListener(
       "gloworm/latencyMillis"
       ,[this](nt::NetworkTable* table
            , std::string_view name
            , nt::NetworkTableEntry entry
            , std::shared_ptr<nt::Value> value
            , int flags)
        { NTcallback(table, name, entry, value, flags); }
        , nt::EntryListenerFlags::kUpdate);
}

void VisionSubsystem::NTcallback(nt::NetworkTable* table, std::string_view name, nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags)
{
    static units::time::second_t ntcallbackTimestamp = Timer::GetFPGATimestamp();
    SmartDashboard::PutNumber("Dt", (Timer::GetFPGATimestamp() - ntcallbackTimestamp).to<double>());
    // printf("Dt: %f\n", (Timer::GetFPGATimestamp() - visionTimestamp).to<double>());
    Work();
    ntcallbackTimestamp = Timer::GetFPGATimestamp();
}

void VisionSubsystem::Periodic()
{
    //Work();
}
    
void VisionSubsystem::Work()
{
    m_visionTimestamp = Timer::GetFPGATimestamp();

    bool bLogInvalid = m_dbgLogInvalid;

    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    bool validTarget = result.HasTargets();
    // printf("valid target %d\n", validTarget);
    if (validTarget)
    {
        vector<frc::Translation2d> targetVectors;
        auto targets = result.GetTargets();
        GetVisionTargetCoords(targets, targetVectors);

 //fprintf(m_logFile, " pitch-filtered targets: %d   ", targetVectors.size());

        if (m_odometry.OdoValid())
        {
            FilterTargets(targetVectors, m_cameraToHub, 12.0_in, degree_t(wpi::numbers::pi/2), degree_t(wpi::numbers::pi/2));//TODO larger than 12 inches? 20-30?
        }
        else
        {
            frc::Translation2d averageTarget = FindAverageOfTargets(targetVectors);
            FilterTargets(targetVectors, averageTarget, kHubRadius * 1.5, degree_t(0.0), degree_t(360.0));
        }

// fprintf(m_logFile, " outlier-filtered targets: %d   ", targetVectors.size());
        if (targetVectors.size() >= 3)
        {
            frc::Translation2d cameraToHub = FitCircle(targetVectors, meter_t{0.01}, 20);
            if (cameraToHub != frc::Translation2d())
            {
                m_consecNoTargets = 0;
                m_validTarget = true;

                Rotation2d fieldToCamRot;
                Translation2d camToRobotCenter;
                GetFieldReleativeRobotAndCameraPoses(cameraToHub, result, fieldToCamRot, camToRobotCenter);

                m_cameraToHub = CompensateMotionForLatency(fieldToCamRot, camToRobotCenter);    // Use odo comp
                //m_cameraToHub = cameraToHub; // Use pure vision

                // printf("latency ms: %.1f delayed odo pose: x %.3f y %.3f   ", 1000*result.GetLatency().to<double>(), delayedOdoPose.X().to<double>(), delayedOdoPose.Y().to<double>());
                // printf("latency ms: %.1f compenstaion: x %.3f y %.3f    ", 1000*result.GetLatency().to<double>(), compenstaion.X().to<double>(), compenstaion.Y().to<double>());
                // printf("compensated camera pose: x %.3f y %.3f\n", m_cameraToHub.X().to<double>(), m_cameraToHub.Y().to<double>());

                // do Hub distance smoothing
                if (m_smoothedRange > 0)
                    m_smoothedRange = kRangeSmoothing * m_smoothedRange + (1 - kRangeSmoothing) * GetHubDistance(false);
                else
                    m_smoothedRange = GetHubDistance(false);
            }
            else
            {
                frc::DataLogManager::Log("Circle fit failed");
                if (bLogInvalid)
                    fprintf(m_logFile, "Circle fit failed \n");
                    //std::cout << "Circle fit failed " << std::endl;
                validTarget =  false;
            }
        }
        else
        {
//            frc::DataLogManager::Log(fmt::format("Only {}  vision targets", targetVectors.size()));
            if (bLogInvalid)
                fprintf(m_logFile, "Only %d  vision targets\n", targetVectors.size());
                //std::cout << fprintf(m_logFile, "Only " << targetVectors.size() << " vision targets" << std::endl;
            validTarget =  false; 
        }
    } // validTarget == true
    else
    { 
        // validTarget == false
        // fprintf(m_logFile, "NO VISION RESULT -- USING ODO ");            
        m_consecNoTargets++;
        if (m_consecNoTargets >= kVisionFailLimit)
        {
            m_validTarget = false;
            m_smoothedRange = 0;
        }
    }

    if (m_odometry.OdoValid())
    {
        m_cameraToHub = Targeting();
    }
    //else if (validTarget == true)  **** // CAN'T INITIALIZE ODO WITH VISION SINCE VISION NEEDS GYRO TO DETERMINE POSE ***** 
    //{
    //     // Init absolute gyro angle isn't required by ResetOdometry() but IS required due to directly reading the gyro elsewhere
    //     m_gyro->SetHeading((double)visionRobotPose.Rotation().Degrees()); 
    //     m_odometry.ResetOdometry(visionRobotPose);
    //     printf("Resetting Odometry from Vision: x=%.3f, y=%.3f, heading =%.1f", m_odometry.GetPose().X().to<double>(), m_odometry.GetPose().Y().to<double>(), m_odometry.GetPose().Rotation().Degrees().to<double>());
    //}

    SmartDashboard::PutNumber("VisionDistance: ", GetHubDistance(false) * 39.37);

    static int turretCmdHoldoff = 0;

    if (m_dbgUseUseVisionForTurret)
    {
        if (turretCmdHoldoff > 0)
        {
            turretCmdHoldoff--;
        }
        else if (m_odometry.OdoValid())
        {
            turretCmdHoldoff = 0; // 3;  // limit turret command rate due to vision lag
            SteerTurretAndAdjusthood();
        }
    }

    static int counter=0;
    if (counter++ % 25 == 0)
    {
        printf("Odometry Pose: x=%.3f, y=%.3f, heading =%.1f\n", m_odometry.GetPose().X().to<double>()* 39.37, m_odometry.GetPose().Y().to<double>()* 39.37, m_odometry.GetPose().Rotation().Degrees().to<double>());
        if (validTarget)
            {
            printf("Vision Pose..: x=%.3f, y=%.3f, heading =%.1f\n", m_robotvisionPose.X().to<double>()* 39.37, m_robotvisionPose.Y().to<double>()* 39.37, m_robotvisionPose.Rotation().Degrees().to<double>());
            printf("camera pose x %.3f y %.3f theta %.3f\n", m_cameraPose.X().to<double>()* 39.37, m_cameraPose.Y().to<double>()* 39.37, m_cameraPose.Rotation().Degrees().to<double>());
            }  
        else
            printf("NO Vision Pose\n");
        //printf(".\n");

        if (!m_validTarget && bLogInvalid)
        {
            //fprintf(m_logFile, "PhotonCam Has No Targets!\n");
            //std::cout << "PhotonCam Has No Targets!" << std::endl;
        }
        else if (m_dbgLogTargetData)
        {
//            fprintf(m_logFile, "Angle: %f, Range: %f, Robot X %f, Y: %f, Theta: %f\n", GetHubAngle() *180/3.14, GetHubDistance(true) * 39.37, m_robotPose.X().to<double>() * 39.37,m_robotPose.Y().to<double>() * 39.37,m_robotPose.Rotation().Degrees().to<double>()); 
            // std::cout << "Center: (" << (double)m_cameraToHub.X() << "," << (double)m_cameraToHub.Y() << "). ";
            // std::cout << "Angle:  " << GetHubAngle() *180/3.14<< ", ";
            // std::cout << "Range: " << GetHubDistance(true) * 39.37 << ", ";
            // std::cout << "Robot X: " << (double) m_robotPose.X() * 39.37 << ", Y: " << (double) m_robotPose.Y() * 39.37 << ", Theta: ", m_robotPose.Rotation().Degrees().to<double>();
            // for(int i = 0; i < targetVectors.size(); i++) {
            //     std::cout << "(" << (double)targetVectors[i].X() << "," << (double)targetVectors[i].Y() << "). ";
            // }
            //std::cout << std::endl;
            
        }
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
            printf("discarded pitch = %f \n", TargetPitch.to<double>());
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

void VisionSubsystem::FilterTargets(vector<frc::Translation2d>& targetVectors, frc::Translation2d center, meter_t rMax, degree_t minangle, degree_t maxangle)
{
    for (size_t i = 0; i < targetVectors.size(); i++)
    {
        Translation2d r = targetVectors[i] - center;

        if (units::math::fabs(r.Norm() - kVisionTargetRadius) > rMax || 
            (GetVectorAngle(r) < units::radian_t{GetVectorAngle(center) + minangle} && GetVectorAngle(r) > units::radian_t{GetVectorAngle(center) - maxangle}))
        {
            targetVectors.erase(targetVectors.begin() + i);
            i--;
            //if (bLogInvalid)
                //std::cout << "Target Discarded" << std::endl; // This floods at 30+ FPS!!!
        }
    }
}

void  VisionSubsystem::GetFieldReleativeRobotAndCameraPoses(frc::Translation2d& cameraToHub, photonlib::PhotonPipelineResult& result, Rotation2d& fieldToCamRot, Translation2d& camToRobotCenter)
{
    // cameraToHub is the vector from cam to hub IN CAMERA-RELATIVE COORDINATE SYSTEM!
    // printf("camera pose from circle fit: x %.3f y %.3f    ", m_cameraToHub.X().to<double>(), m_cameraToHub.Y().to<double>());
    second_t visionTimestamp = m_visionTimestamp - result.GetLatency();
    StateHist delayedState = m_odometry.GetState(visionTimestamp);
    frc::Pose2d delayedOdoPose = delayedState.pose;
    degree_t angleTurret = delayedState.m_turretAngle;
    Rotation2d robotRot = delayedOdoPose.Rotation(); // robot heading FIELD RELATIVE
    fieldToCamRot = robotRot + Rotation2d(angleTurret + 180_deg);  

    Translation2d camToTurretCenterRRC = Translation2d(-5_in, 0_in).RotateBy(Rotation2d{angleTurret});  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenterRRC = camToTurretCenterRRC + turretCenterToRobotCenter;  // ROBOT RELATIVE COORDINATES
    camToRobotCenter = camToRobotCenterRRC.RotateBy(robotRot);  // FIELD RELATIVE COORDINATES

    m_cameraPose = Pose2d(kHubCenter - cameraToHub.RotateBy(fieldToCamRot), fieldToCamRot); // FIELD RELATIVE cam pose
    
    m_robotvisionPose = Pose2d(m_cameraPose.Translation() + camToRobotCenter, robotRot);  // FIELD RELATIVE robot pose
    // printf("camera pose x %.3f y %.3f theta %.3f   ", cameraPose.X().to<double>(), cameraPose.Y().to<double>(), cameraPose.Rotation().Degrees().to<double>());
    // printf("robot pose x %.3f y %.3f theta %.3f   ", robotvisionPose.X().to<double>(), robotvisionPose.Y().to<double>(), robotvisionPose.Rotation().Degrees().to<double>());
}

Translation2d  VisionSubsystem::CompensateMotionForLatency(Rotation2d& fieldToCamRot, Translation2d& camToRobotCenter)
{
    // Use wheel odo to correct robotvisionPose for movement since image was captured
    //frc::Pose2d lastOdoState = m_odometry.GetPose(); // auto& lastOdoState = m_odometry.GetStateHist().back();  
    // frc::Transform2d compenstaion = Transform2d(lastOdoState.pose, delayedOdoPose);

    Transform2d compenstaion; // zero transform for testing
    Pose2d compensatedRobotvisionPose = m_robotvisionPose.TransformBy(compenstaion);             

    Pose2d compensatedCameraPose = Pose2d(compensatedRobotvisionPose.Translation() - camToRobotCenter, fieldToCamRot);  // FIELD RELATIVE COORDINATES    
    Translation2d cameraToHubFR = kHubCenter - compensatedCameraPose.Translation(); // FIELD RELATIVE COORDINATES    

    return cameraToHubFR.RotateBy(-fieldToCamRot); // transform from field-relative back to cam-relative
}

Translation2d  VisionSubsystem::Targeting()
{
    // use odometry instead of vision
    StateHist lastOdoState = m_odometry.GetState();
    degree_t angleTurret = lastOdoState.m_turretAngle;

    m_robotPose = m_odometry.GetPose();//lastOdoState.pose;
    // frc::Translation2d camToTurretCenter = frc::Translation2d(meter_t{(cos(angleTurret) * inch_t{-12})}, meter_t{(sin(angleTurret) * inch_t{-12})});
    // frc::Transform2d camreaTransform = frc::Transform2d(camToTurretCenter + turretCenterToRobotCenter, radian_t{angleTurret});
    // frc::Rotation2d fieldToCamAngle = m_robotPose.Rotation() + frc::Rotation2d(units::radian_t{angleTurret});  
    // m_cameraToHub = kHubCenter - m_robotPose.TransformBy(camreaTransform.Inverse()).Translation();

    Rotation2d robotRot = m_robotPose.Rotation(); // robot heading FIELD RELATIVE
    Rotation2d fieldToCamRot = robotRot + Rotation2d(angleTurret + 180_deg);  
    Translation2d camToTurretCenterRRC = Translation2d(-5_in, 0_in).RotateBy(Rotation2d{angleTurret});  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenterRRC = camToTurretCenterRRC + turretCenterToRobotCenter;  // ROBOT RELATIVE COORDINATES
    Translation2d camToRobotCenter = camToRobotCenterRRC.RotateBy(robotRot);  // FIELD RELATIVE COORDINATES                
    Pose2d cameraPose = Pose2d(m_robotPose.Translation() - camToRobotCenter, fieldToCamRot);  // FIELD RELATIVE COORDINATES    
    Translation2d cameraToHubFR = kHubCenter - cameraPose.Translation(); // FIELD RELATIVE COORDINATES    

    return cameraToHubFR.RotateBy(-fieldToCamRot); // transform from field-relative back to cam-relative
}


void  VisionSubsystem::SteerTurretAndAdjusthood()
{
    auto hubAngle = GetHubAngle() * 180.0 / wpi::numbers::pi;
    m_turret.TurnToRelative(hubAngle * 1.0); // can apply P constant < 1.0 if needed for vision tracking stability 
    m_hood.SetByDistance(GetHubDistance(false));
    //printf("Turret Angle %.2f   ", m_turret.GetCurrentAngle());
    //printf("Hub Angle: %.2f \n", hubAngle);
    printf( " Hub angle: %f  range: %f\n", GetHubAngle()*180/3.14159, GetHubDistance(true)*39.37);
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

