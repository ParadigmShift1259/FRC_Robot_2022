
#include "subsystems/VisionSubsystem.h"
#include <iostream>
#include <vector>
#include <photonlib/PhotonUtils.h>

VisionSubsystem::VisionSubsystem(Gyro2 *gyro, TurretSubsystem *turret) 
 : m_dashboard (nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard"))
 , m_networktable(nt::NetworkTableInstance::GetDefault().GetTable("gloworm"))
 , m_led(true)
 , m_validTarget(false)
 , m_gyro(gyro)
 , m_turret(turret)
{
    SetLED(true);
    m_averageDistance.reserve(3);
    m_averageAngle.reserve(3);
    kCameraHeight = inch_t{41.25}; //37
    kCurrTargetHeight = inch_t{8*12 + 7};
    kCameraPitch = degree_t{21.0}; // 18.8
    kTargetPitch = degree_t{0};
    m_consecNoTargets = 0;

   // m_networktable->AddEntryListener(NTcallback, nt::EntryListenerFlags::kUpdate);
}

//void VisionSubsystem::NTcallback(nt::NetworkTable* table, std::string_view name, nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags))
void VisionSubsystem::Periodic()
{
    static unsigned counter = 0; 
    counter++;
    bool bLogInvalid = m_dbgLogInvalid;
    bool willPrint = false;
    if (counter % 25 == 0)
        willPrint = true;

    camera.SetLEDMode(photonlib::LEDMode::kDefault);

    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    bool validTarget = result.HasTargets();
    vector<Translation2d> targetVectors;
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
        Translation2d averageTarget = Translation2d(meter_t{xMean}, meter_t{yMean});

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
            if (FitCircle(targetVectors, meter_t{0.01}, 20))
            {
                if (m_smoothedRange > 0)
                    m_smoothedRange = kRangeSmoothing * m_smoothedRange + (1 - kRangeSmoothing) * GetHubDistance(false);
                else
                    m_smoothedRange = GetHubDistance(false);
                m_consecNoTargets = 0;
                m_validTarget = true;
                double distToHub = GetHubDistance(false);
                double hubAngle = GetHubAngle();
                double angleTurret = m_turret->GetCurrentAngle();
                double angleToHub = m_gyro->GetHeading() + angleTurret - hubAngle;
                Translation2d displacement = Translation2d(meter_t{(cos(angleToHub) * distToHub)}, meter_t{(sin(angleToHub) * distToHub)});
                Translation2d kHubCenter = Translation2d(kFieldLength/2, kFieldWidth/2);
                Rotation2d turretRot = Rotation2d(radian_t{angleTurret});
                Pose2d cameraPose = Pose2d(kHubCenter-displacement, m_gyro->GetHeadingAsRot2d() + turretRot);
                Translation2d turretCenterToRobotCenter = Translation2d(inch_t{2.25}, inch_t{0});
                Translation2d camToTurretCenter = Translation2d(meter_t{(cos(angleTurret) * inch_t{-12})}, meter_t{(sin(angleTurret) * inch_t{-12})});
                Transform2d camreaTransform = Transform2d(camToTurretCenter + turretCenterToRobotCenter, radian_t{-m_turret->GetCurrentAngle()});
                m_robotPose = cameraPose.TransformBy(camreaTransform);
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
            m_smoothedRange = 0;
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
        else if (m_validTarget)
        {
            auto hubAngle = GetHubAngle() * 180.0 / wpi::numbers::pi;
            m_turret->TurnToRelative(hubAngle * 1);
            turretCmdHoldoff = 3;  // limit turret command rate due to vision lag
        }
    }
    else
    {
        m_turret->TurnToField(0.0);
    }

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


bool VisionSubsystem::FitCircle(vector<frc::Translation2d> targetVectors, meter_t precision, int maxAttempts)
{
    double xSum = 0.0;
    double ySum = 0.0;
    for (int i = 0; i < targetVectors.size(); i++) 
    {
        xSum += (double) targetVectors[i].X();
        ySum += (double) targetVectors[i].Y();
    }
    frc::Translation2d cameraToHub = Translation2d(meter_t{xSum / targetVectors.size()} + kVisionTargetRadius, meter_t{ySum / targetVectors.size()});

    // Iterate to find optimal center
    meter_t shiftDist = kVisionTargetRadius / 2.0;
    meter_t minResidual = calcResidual(kVisionTargetRadius, targetVectors, cameraToHub);

    int n = 0;

    while (n < maxAttempts) {
        vector<frc::Translation2d> translations;
        translations.push_back(Translation2d(shiftDist, meter_t{0.0}));
        translations.push_back(Translation2d(-shiftDist, meter_t{0.0}));
        translations.push_back(Translation2d(meter_t{0.0},shiftDist));
        translations.push_back(Translation2d(meter_t{0.0},-shiftDist));
        frc::Translation2d bestPoint = cameraToHub;
        bool centerIsBest = true;

        // Check all adjacent positions
        for (int i = 0; i < translations.size(); i++) 
        {
            meter_t residual =
                calcResidual(kVisionTargetRadius, targetVectors, cameraToHub + (translations[i]));
            if (residual < minResidual) {
                bestPoint = cameraToHub + (translations[i]);
                minResidual = residual;
                centerIsBest = false;
                break;
            }
        }
        // Decrease shift, exit, or continue
        if (centerIsBest) {
            shiftDist /= 2.0;
            if (shiftDist < precision) {
                m_cameraToHub = cameraToHub;
                return true;
            }
        } else {
            cameraToHub = bestPoint;
        }

        n++;
    }
    // failed
    return false;
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
    if (smoothed)
        return m_smoothedRange;

    return (double) m_cameraToHub.Norm();
}

