
#include "subsystems/VisionSubsystem.h"
#include <units/math.h>
#include <iostream>
#include <vector>
#include <photonlib/PhotonUtils.h>

VisionSubsystem::VisionSubsystem(Team1259::Gyro *gyro, TurretSubsystem& turret, HoodSubsystem& hood, IOdometry& odometry) 
 : m_networktable(nt::NetworkTableInstance::GetDefault().GetTable("gloworm"))
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
       "photonvision/gloworm/latencyMillis"
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
    //Work();
}

void VisionSubsystem::Periodic()
{
    Work();
}
    
void VisionSubsystem::Work()
{
    static Timer timer;
    units::time::second_t visionTimestamp = timer.GetFPGATimestamp();

    //static unsigned counter = 0; 
    bool bLogInvalid = m_dbgLogInvalid;

    const frc::Translation2d kHubCenter = frc::Translation2d(kFieldLength/2, kFieldWidth/2);  // TO DO make a constant
    const frc::Translation2d turretCenterToRobotCenter = frc::Translation2d(inch_t{2.25}, inch_t{0});   // TO DO make a constant
 
    Pose2d visionRobotPose;

    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    bool validTarget = result.HasTargets();
//printf("valid target %d\n", validTarget);
    if (validTarget)
    {
        vector<frc::Translation2d> targetVectors;
        auto targets = result.GetTargets();

// fprintf(m_logFile, " target count: %d   ",  targets.size());
        // Gets camera-relative x,y translations for each vision target
        for (size_t i = 0; i < targets.size(); i++)
        {
            degree_t TargetPitch = degree_t{targets[i].GetPitch()};
            meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
                kCameraHeight, kCurrTargetHeight, kCameraPitch, TargetPitch);
            if (!(TargetPitch > units::degree_t{24}) && !(TargetPitch < units::degree_t{-9}))
                targetVectors.push_back(photonlib::PhotonUtils::EstimateCameraToTargetTranslation(range, frc::Rotation2d(degree_t{-targets[i].GetYaw()})));
        }

// fprintf(m_logFile, " pitch-filtered targets: %d   ", targetVectors.size());


        // //find the center of the vision targets
        // double xTotal = 0;
        // double yTotal = 0;
        // for (size_t i = 0; i < targetVectors.size(); i++)
        // {
        //     xTotal += (double)targetVectors[i].X();
        //     yTotal += (double)targetVectors[i].Y();
        // }
        // double xMean = xTotal/targetVectors.size();
        // double yMean = yTotal/targetVectors.size();
        // frc::Translation2d averageTarget = Translation2d(meter_t{xMean}, meter_t{yMean});

        //Throw out outliers
        for (size_t i = 0; i < targetVectors.size(); i++)
        {
            units::meter_t rTolerance = 12.0_in;
            
            Translation2d r = targetVectors[i] - m_cameraToHub;

            if (units::math::fabs(r.Norm() - kVisionTargetRadius) > rTolerance || 
                (GetVectorAngle(r) < units::radian_t{GetHubAngle() + wpi::numbers::pi/2} && GetVectorAngle(r) > units::radian_t{GetHubAngle() - wpi::numbers::pi/2}))
            {
                targetVectors.erase(targetVectors.begin() + i);
                i--;
                //if (bLogInvalid)
                    //std::cout << "Target Discarded" << std::endl; // This floods at 30+ FPS!!!
            }
        }

// fprintf(m_logFile, " outlier-filtered targets: %d   ", targetVectors.size());

        if (targetVectors.size() >= 3)
        {
            frc::Translation2d cameraToHub = FitCircle(targetVectors, meter_t{0.01}, 20);
            if (cameraToHub != frc::Translation2d())
            {
                m_consecNoTargets = 0;
                m_validTarget = true;
                // cameraToHub is the vector from cam to hub IN CAMERA-RELATIVE COORDINATE SYSTEM!
                // printf("camera pose from circle fit: x %.3f y %.3f    ", m_cameraToHub.X().to<double>(), m_cameraToHub.Y().to<double>());
#define USE_ODO_COMPENSATION
#ifdef USE_ODO_COMPENSATION
                visionTimestamp = visionTimestamp - result.GetLatency();
                frc::Pose2d delayedOdoPose = m_odometry.GetPose(visionTimestamp);
                
                double angleTurret = Util::DegreesToRadians(m_turret.GetCurrentAngle()); // TO DO keep history of turret angle and use that instead of current turrent angle
                Rotation2d fieldToCamAngle = delayedOdoPose.Rotation() + Rotation2d(radian_t{angleTurret});  // TO DO keep history of turret angle and use that instead of current turrent angle
                frc::Translation2d camToTurretCenter = frc::Translation2d(meter_t{(cos(angleTurret) * inch_t{-12})}, meter_t{(sin(angleTurret) * inch_t{-12})});
                frc::Transform2d camreaTransform = frc::Transform2d(camToTurretCenter + turretCenterToRobotCenter, radian_t{angleTurret});

                Pose2d cameraPose = Pose2d(kHubCenter - cameraToHub.RotateBy(fieldToCamAngle), fieldToCamAngle); // field relative cam pose
                auto RobotvisionPose = cameraPose.TransformBy(camreaTransform.Inverse());  // where vision thinks robot was when image was captured (e.g. latency)
                // printf("camera pose x %.3f y %.3f theta %.3f   ", cameraPose.X().to<double>(), cameraPose.Y().to<double>(), cameraPose.Rotation().Degrees().to<double>());
                // printf("robot pose x %.3f y %.3f theta %.3f   ", RobotvisionPose.X().to<double>(), RobotvisionPose.Y().to<double>(), RobotvisionPose.Rotation().Degrees().to<double>());

                // Use wheel odo to correct RobotvisionPose for movement since image was captured
                //frc::Pose2d lastOdoState = m_odometry.GetPose(); // auto& lastOdoState = m_odometry.GetStateHist().back();  
                // frc::Transform2d compenstaion = Transform2d(lastOdoState.pose, delayedOdoPose);


                frc::Transform2d compenstaion; // zero transform for testing
                visionRobotPose = RobotvisionPose.TransformBy(compenstaion);              
                m_cameraToHub = kHubCenter - visionRobotPose.TransformBy(camreaTransform).Translation();
                m_cameraToHub = m_cameraToHub.RotateBy(-fieldToCamAngle); // transform from field-relative back to cam-relative

                // printf("latency ms: %.1f delayed odo pose: x %.3f y %.3f   ", 1000*result.GetLatency().to<double>(), delayedOdoPose.X().to<double>(), delayedOdoPose.Y().to<double>());
                // printf("latency ms: %.1f compenstaion: x %.3f y %.3f    ", 1000*result.GetLatency().to<double>(), compenstaion.X().to<double>(), compenstaion.Y().to<double>());
                // printf("compensated camera pose: x %.3f y %.3f\n", m_cameraToHub.X().to<double>(), m_cameraToHub.Y().to<double>());
#else
                m_cameraToHub = cameraToHub;
                // printf("camera pose x %.3f y %.3f\n", m_cameraToHub.X().to<double>(), m_cameraToHub.Y().to<double>());

#endif  // def USE_ODO_COMPENSATION
                // do Hub distance smoothing
                if (m_smoothedRange > 0)
                    m_smoothedRange = kRangeSmoothing * m_smoothedRange + (1 - kRangeSmoothing) * GetHubDistance(false);
                else
                    m_smoothedRange = GetHubDistance(false);
            }
            else
            {
                if (bLogInvalid)
                    fprintf(m_logFile, "Circle fit failed \n");
                    //std::cout << "Circle fit failed " << std::endl;
                validTarget =  false;
            }
        }
        else
        {
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
            // use odometry instead of vision
            m_robotPose = m_odometry.GetPose();
            double angleTurret = Util::DegreesToRadians(m_turret.GetCurrentAngle());
            frc::Translation2d camToTurretCenter = frc::Translation2d(meter_t{(cos(angleTurret) * inch_t{-12})}, meter_t{(sin(angleTurret) * inch_t{-12})});
            frc::Transform2d camreaTransform = frc::Transform2d(camToTurretCenter + turretCenterToRobotCenter, radian_t{angleTurret});
            frc::Rotation2d fieldToCamAngle = m_robotPose.Rotation() + frc::Rotation2d(units::radian_t{angleTurret});  // TO DO keep history of turret angle and use that instead of current turrent angle    
            m_cameraToHub = kHubCenter - m_robotPose.TransformBy(camreaTransform.Inverse()).Translation();
            m_cameraToHub = m_cameraToHub.RotateBy(-fieldToCamAngle); // transform from field-relative back to cam-relative
            }
        // else if (validTarget == true)  **** // CAN'T INITIALIZE ODO WITH VISION SINCE VISION NEEDS GYRO TO DETERMINE POSE ***** 
        //     {
        //     // Init absolute gyro angle isn't required by ResetOdometry() but IS required due to directly reading the gyro elsewhere
        //     m_gyro->SetHeading((double)visionRobotPose.Rotation().Degrees()); 
        //     m_odometry.ResetOdometry(visionRobotPose);
        //     printf("Resetting Odometry from Vision: x=%.3f, y=%.3f, heading =%.1f", m_odometry.GetPose().X().to<double>(), m_odometry.GetPose().Y().to<double>(), m_odometry.GetPose().Rotation().Degrees().to<double>());
        //     }

    SmartDashboard::PutNumber("VisionDistance: ", GetHubDistance(false));

    static int turretCmdHoldoff = 0;

    if (m_dbgUseUseVisionForTurret)
    {
        if (turretCmdHoldoff > 0)
        {
            turretCmdHoldoff--;
        }
        else if (m_odometry.OdoValid())
        {
            auto hubAngle = GetHubAngle() * 180.0 / wpi::numbers::pi;
            m_turret.TurnToRelative(hubAngle * 1.0);
            turretCmdHoldoff = 2;  // limit turret command rate due to vision lag
            m_hood.SetByDistance(GetHubDistance(true));
            //printf("Turret Angle %.2f   ", m_turret.GetCurrentAngle());
            //printf("Hub Angle: %.2f \n", hubAngle);
            // printf( " Hub angle: %f  range: %f\n", GetHubAngle(), GetHubDistance(true));
        }
    }

    // if (counter++ % 25 == 0)
    // {
    //     printf("Odometry Pose: x=%.3f, y=%.3f, heading =%.1f\n", m_odometry.GetPose().X().to<double>(), m_odometry.GetPose().Y().to<double>(), m_odometry.GetPose().Rotation().Degrees().to<double>());
    //     if (validTarget)
    //         printf("Vision Pose..: x=%.3f, y=%.3f, heading =%.1f\n", visionRobotPose.X().to<double>(), visionRobotPose.Y().to<double>(), visionRobotPose.Rotation().Degrees().to<double>());
    //     else
    //         printf("NO Vision Pose");
    //     printf(".\n");

    //     if (!m_validTarget && bLogInvalid)
    //     {
    //         fprintf(m_logFile, "PhotonCam Has No Targets!\n");
    //         //std::cout << "PhotonCam Has No Targets!" << std::endl;
    //     }
    //     else if (m_dbgLogTargetData)
    //     {
    //         fprintf(m_logFile, "Angle: %f, Range: %f, Robot X %f, Y: %f, Theta: %f\n", GetHubAngle() *180/3.14, GetHubDistance(true) * 39.37, m_robotPose.X().to<double>() * 39.37,m_robotPose.Y().to<double>() * 39.37,m_robotPose.Rotation().Degrees().to<double>()); 
    //         // std::cout << "Center: (" << (double)m_cameraToHub.X() << "," << (double)m_cameraToHub.Y() << "). ";
    //         // std::cout << "Angle:  " << GetHubAngle() *180/3.14<< ", ";
    //         // std::cout << "Range: " << GetHubDistance(true) * 39.37 << ", ";
    //         // std::cout << "Robot X: " << (double) m_robotPose.X() * 39.37 << ", Y: " << (double) m_robotPose.Y() * 39.37 << ", Theta: ", m_robotPose.Rotation().Degrees().to<double>();
    //         // for(int i = 0; i < targetVectors.size(); i++) {
    //         //     std::cout << "(" << (double)targetVectors[i].X() << "," << (double)targetVectors[i].Y() << "). ";
    //         // }
    //         //std::cout << std::endl;
            
    //     }
    // }
 
    SmartDashboard::PutNumber("D_V_Active", m_validTarget);
    // SmartDashboard::PutNumber("D_V_Distance", distance);
    // SmartDashboard::PutNumber("D_V_Angle", m_horizontalangle);
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

