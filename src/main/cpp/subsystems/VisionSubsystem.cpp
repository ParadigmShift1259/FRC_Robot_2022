
#include "subsystems/VisionSubsystem.h"
#include <iostream>
#include <vector>
#include <photonlib/PhotonUtils.h>

units::meter_t kVisionHubOffsetRimToCenter = units::foot_t(2.0);

VisionSubsystem::VisionSubsystem(Team1259::Gyro *gyro, TurretSubsystem& turret, HoodSubsystem& hood, IOdometry& odometry) 
 //: m_dashboard (nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard"))
 : m_networktable(nt::NetworkTableInstance::GetDefault().GetTable("gloworm"))
 , m_led(true)
 , m_validTarget(false)
 , m_gyro(gyro)
 , m_turret(turret)
 , m_hood(hood)
 , m_odometry(odometry)
{
    SetLED(true);
    m_averageDistance.reserve(3);
    m_averageAngle.reserve(3);
    kCameraHeight = inch_t{41.25}; //37
    kCurrTargetHeight = inch_t{8*12 + 7};
    kCameraPitch = degree_t{21.0}; // 18.8
    kTargetPitch = degree_t{0};
    m_consecNoTargets = 0;

//    m_networktable->AddEntryListener(
//        "/photonvision/gloworm/latencyMillis"
//        ,[this](nt::NetworkTable* table
//             , std::string_view name
//             , nt::NetworkTableEntry entry
//             , std::shared_ptr<nt::Value> value
//             , int flags)
//         { NTcallback(table, name, entry, value, flags); }
//         , nt::EntryListenerFlags::kUpdate);
}

//void VisionSubsystem::NTcallback(nt::NetworkTable* table, std::string_view name, nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags)
void VisionSubsystem::Periodic()
{
    static Timer timer;
    units::time::second_t visionTimestamp = timer.GetFPGATimestamp();

    static unsigned counter = 0; 
    counter++;
    bool bLogInvalid = m_dbgLogInvalid;
    bool willPrint = false;

    if (counter % 25 == 0)
        willPrint = true;
    
    const frc::Translation2d kHubCenter = frc::Translation2d(kFieldLength/2, kFieldWidth/2);  // TO DO make a constant
    const frc::Translation2d turretCenterToRobotCenter = frc::Translation2d(inch_t{2.25}, inch_t{0});   // TO DO make a constant
 
    camera.SetLEDMode(photonlib::LEDMode::kDefault);

    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    bool validTarget = result.HasTargets();
    vector<frc::Translation2d> targetVectors;
    if (validTarget)
    {
        auto targets = result.GetTargets();

        //Get List of Points
        //if(!m_allPoints.empty())
        
        //Gets Vectors for each target
        for (int i = 0; i < targets.size(); i++)
        {
            kTargetPitch = degree_t{targets[i].GetPitch()};
            meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
                kCameraHeight, kCurrTargetHeight, kCameraPitch, kTargetPitch);
            if (!(kTargetPitch > units::degree_t{24}) && !(kTargetPitch < units::degree_t{-9}))
                targetVectors.push_back(photonlib::PhotonUtils::EstimateCameraToTargetTranslation(range, frc::Rotation2d(degree_t{-targets[i].GetYaw()})));
        }

        //find the center of the vision tape targets
        double xTotal = 0;
        double yTotal = 0;
        for (int i = 0; i < targetVectors.size(); i++)
        {
            xTotal += (double)targetVectors[i].X();
            yTotal += (double)targetVectors[i].Y();
        }
        double xMean = xTotal/targetVectors.size();
        double yMean = yTotal/targetVectors.size();
        frc::Translation2d averageTarget = Translation2d(meter_t{xMean}, meter_t{yMean});

        //Throw out outliers
        for (int i = 0; i < targetVectors.size(); i++)
        {
            if (averageTarget.Distance(targetVectors[i]) > meter_t{kMaxTargetSpread})
            {
                targetVectors.erase(targetVectors.begin() + i);
                i--;
                if (bLogInvalid)
                    std::cout << "Target Discarded" << std::endl; // This floods at 30+ FPS!!!
            }
        }

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
                frc::Pose2d lastOdoState = m_odometry.GetPose(); // auto& lastOdoState = m_odometry.GetStateHist().back();  
                // frc::Transform2d compenstaion = Transform2d(lastOdoState.pose, delayedOdoPose);
                frc::Transform2d compenstaion; // zero transform for testing
                m_robotPose = RobotvisionPose.TransformBy(compenstaion);              
                m_cameraToHub = kHubCenter - m_robotPose.TransformBy(camreaTransform).Translation();
                m_cameraToHub = m_cameraToHub.RotateBy(-fieldToCamAngle); // transform from field-relative back to cam-relative

                // printf("latency ms: %.1f delayed odo pose: x %.3f y %.3f   ", 1000*result.GetLatency().to<double>(), delayedOdoPose.X().to<double>(), delayedOdoPose.Y().to<double>());
                printf("latency ms: %.1f compenstaion: x %.3f y %.3f    ", 1000*result.GetLatency().to<double>(), compenstaion.X().to<double>(), compenstaion.Y().to<double>());
                printf("compensated camera pose: x %.3f y %.3f\n", m_cameraToHub.X().to<double>(), m_cameraToHub.Y().to<double>());
#else
                m_cameraToHub = cameraToHub;
                printf("camera pose x %.3f y %.3f\n", m_cameraToHub.X().to<double>(), m_cameraToHub.Y().to<double>());

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
                    std::cout << "Circle fit failed " << std::endl;
                m_consecNoTargets++;
            }
        }
        else
        {
            if (bLogInvalid)
                std::cout << "Only " << targetVectors.size() << " vision targets" << std::endl;
            m_consecNoTargets++;
        }

        if (m_consecNoTargets >= kVisionFailLimit)
        {
            m_validTarget = false;
            // m_smoothedRange = 0;
            m_robotPose = m_odometry.GetPose();
            double angleTurret = Util::DegreesToRadians(m_turret.GetCurrentAngle());
            frc::Translation2d camToTurretCenter = frc::Translation2d(meter_t{(cos(angleTurret) * inch_t{-12})}, meter_t{(sin(angleTurret) * inch_t{-12})});
            frc::Transform2d camreaTransform = frc::Transform2d(camToTurretCenter + turretCenterToRobotCenter, radian_t{angleTurret});
            frc::Rotation2d fieldToCamAngle = m_robotPose.Rotation() + frc::Rotation2d(units::radian_t{angleTurret});  // TO DO keep history of turret angle and use that instead of current turrent angle    
            m_cameraToHub = kHubCenter - m_robotPose.TransformBy(camreaTransform.Inverse()).Translation();
            m_cameraToHub = m_cameraToHub.RotateBy(-fieldToCamAngle); // transform from field-relative back to cam-relative
        }
    }

    SmartDashboard::PutNumber("VisionDistance: ", GetHubDistance(false));

    static int turretCmdHoldoff = 0;

    if (m_dbgUseUseVisionForTurret)
    {
        if (turretCmdHoldoff > 0)
        {
            turretCmdHoldoff--;
        }
        else //if (m_validTarget)
        {
            auto hubAngle = GetHubAngle() * 180.0 / wpi::numbers::pi;
            m_turret.TurnToRelative(hubAngle * 1);
            turretCmdHoldoff = 0;  // limit turret command rate due to vision lag
            AdjustHood();
        }
    }
    // else
    // {
    //     m_turret.TurnToField(0.0);
    // }

    if (willPrint)
    {
        if (!m_validTarget && bLogInvalid)
        {
            std::cout << "PhotonCam Has No Targets!" << std::endl;
        }
        else if (m_dbgLogTargetData)
        {
            // std::cout << "Center: (" << (double)m_cameraToHub.X() << "," << (double)m_cameraToHub.Y() << "). ";
            std::cout << "Angle:  " << GetHubAngle() *180/3.14<< ", ";
            std::cout << "Range: " << GetHubDistance(true) * 39.37 << ", ";
            std::cout << "Robot X: " << (double) m_robotPose.X() * 39.37 << ", Y: " << (double) m_robotPose.Y() * 39.37 << ", Theta: ", (double) m_robotPose.Rotation().Degrees();
            // for(int i = 0; i < targetVectors.size(); i++) {
            //     std::cout << "(" << (double)targetVectors[i].X() << "," << (double)targetVectors[i].Y() << "). ";
            // }
            std::cout << std::endl;
        }
    }
 
    SmartDashboard::PutNumber("D_V_Active", m_validTarget);
    // SmartDashboard::PutNumber("D_V_Distance", distance);
    // SmartDashboard::PutNumber("D_V_Angle", m_horizontalangle);
}


bool VisionSubsystem::GetValidTarget()
{
    return m_validTarget;
}

double VisionSubsystem::GetDistance()
{
    return Util::GetAverage(m_averageDistance);
}

double VisionSubsystem::GetAngle()
{
    return Util::GetAverage(m_averageAngle);
}

void VisionSubsystem::SetLED(bool on)
{
    m_led = on;
    if (m_led)
    {
        /// 3 forces limelight led on
        m_networktable->PutNumber("ledMode", 3);
    }
    else
    {
        /// 1 forces limelight led off
        m_networktable->PutNumber("ledMode", 1);
    }
}


frc::Translation2d VisionSubsystem::FitCircle(vector<frc::Translation2d> targetVectors, meter_t precision, int maxAttempts)
{
    double xSum = 0.0;
    double ySum = 0.0;
    for (int i = 0; i < targetVectors.size(); i++) 
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
    for (int i = 0; i < points.size(); i++) {
      double diff = (double) (points[i].Distance(center) - radius);
      residual += diff * diff;
    }
    return meter_t{residual};
}

double VisionSubsystem::GetHubAngle()
{
    return atan2((double)m_cameraToHub.Y(), (double)m_cameraToHub.X());
}

double VisionSubsystem::GetHubDistance(bool smoothed)
{
    if (smoothed && m_smoothedRange > 0)
        return m_smoothedRange;

    return (double) m_cameraToHub.Norm();
}

void VisionSubsystem::AdjustHood()
{
    double distance = (GetHubDistance(true));
    if (distance > 0)
    {
        m_calculation.CalcInitRPMs(meter_t(distance), kVisionHubOffsetRimToCenter);
        degree_t initAngle = m_calculation.GetInitAngle();
        double x = initAngle.to<double>();
        double c = SmartDashboard::GetNumber("Hoodangle Constant", 0.0317);
        double hoodangle = 0.33 - c * x + 0.000816 * x * x;
        if (hoodangle != hoodangle)
        {
            hoodangle = 0.33 - 0.0317 * x + 0.000816 * x * x;
        }
        hoodangle = std::clamp(hoodangle, HoodConstants::kMin, HoodConstants::kMax);
        m_hood.Set(hoodangle);
    }
}
