#include "ConstantsVision.h"
#include "subsystems/VisionSubsystem.h"
#include "common/Util.h"
#include <iostream>
#include <vector>
#include <photonlib/PhotonUtils.h>

VisionSubsystem::VisionSubsystem(Team1259::Gyro *gyro) 
 : m_dashboard (nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard"))
 , m_networktable(nt::NetworkTableInstance::GetDefault().GetTable("gloworm"))
 , m_led(true)
 , m_tx(0)
 , m_ty(0)
 , m_validTarget(false)
 , m_gyro(gyro)
{
    SetLED(true);
    m_averageDistance.reserve(3);
    m_averageAngle.reserve(3);
    kCameraHeight = units::inch_t{41.25}; //37
    kCurrTargetHeight = units::inch_t{8*12 + 7};
    kCameraPitch = units::degree_t{21.0}; // 18.8
    kTargetPitch = units::degree_t{0};
}

void VisionSubsystem::Periodic()
{

    static unsigned counter = 0; 
    counter++;
    bool willPrint = false;
    if (counter % 50 == 0)
        willPrint = true;

    camera.SetLEDMode(photonlib::LEDMode::kDefault);

    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    bool m_validTarget = result.HasTargets();
    vector<frc::Translation2d> targetVectors;
    if (m_validTarget)
    {
        auto targets = result.GetTargets();

        //Get List of Points
        //if(!m_allPoints.empty())
        
        //Gets Vectors for each target
        for(int i = 0; i < (int)targets.size(); i++)
        {
            kTargetPitch = units::degree_t{targets[i].GetPitch()};
            units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
                kCameraHeight, kCurrTargetHeight, kCameraPitch, kTargetPitch);
            targetVectors.push_back(photonlib::PhotonUtils::EstimateCameraToTargetTranslation(range, frc::Rotation2d(units::degree_t{-targets[i].GetYaw()})));
        }

        //find the center of the vision tape targets
        double xTotal = 0;
        double yTotal = 0;
        for(int i = 0; i < targetVectors.size(); i++)
        {
            xTotal += (double)targetVectors[i].X();
            yTotal += (double)targetVectors[i].Y();
        }
        double xMean = xTotal/targetVectors.size();
        double yMean = yTotal/targetVectors.size();
        frc::Translation2d averageTarget = Translation2d(units::meter_t{xMean}, units::meter_t{yMean});

        //Throw out outliers
        for (auto it = targetVectors.begin(); it != targetVectors.end(); ++it)
        {
            if (averageTarget.Distance(*it) > units::meter_t{kMaxTargetSpread})
            {
                targetVectors.erase(it);
                std::cout << "Target Discarded" << std::endl;
            }
        }

        if(targetVectors.size() >= 3)
        {
            FitCircle(targetVectors); // To Do: Pass in Precision and Iteration Limit
        }
        else
        {
            std::cout << "Insufficient Targets for Circle Fit" << std::endl;
            m_validTarget = false;
        }
    }

    // frc::Pose2d TargetPose = Pose2d(units::inch_t{324}, units::inch_t{162}, frc::Rotation2d{units::degree_t{0}});
    // units::radian_t angleToTarget = units::radian_t(atan2(double(m_cameraToHub.X()), double(m_cameraToHub.Y())));
    // frc::Pose2d CamPose = photonlib::PhotonUtils::EstimateFieldToCamera(frc::Transform2d(m_cameraToHub, angleToTarget), TargetPose);
    // frc::Transform2d CamToTarget = photonlib::PhotonUtils::EstimateCameraToTarget(m_cameraToHub, TargetPose, m_gyro->GetHeadingAsRot2d());
    // frc::Transform2d CamToRobot = Transform2d();
    // frc::Pose2d robotPose = photonlib::PhotonUtils::EstimateFieldToRobot(CamToTarget, TargetPose, CamToRobot);
    if(willPrint)
        {
        if (!m_validTarget)
            std::cout << "PhotonCam Has No Targets!" << std::endl;
        else
            {
                std::cout << "Center: (" << (double)m_cameraToHub.X() << "," << (double)m_cameraToHub.Y() << "). ";
                std::cout << "Target Vectors: ";
                for(int i = 0; i < targetVectors.size(); i++) {
                    std::cout << "(" << (double)targetVectors[i].X() << "," << (double)targetVectors[i].Y() << "). ";
                }
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


bool VisionSubsystem::FitCircle(vector<frc::Translation2d> targetVectors)
{
    double xSum = 0.0;
    double ySum = 0.0;
    for (int i = 0; i < (int)targetVectors.size(); i++) 
    {
        xSum += (double) targetVectors[i].X();
        ySum += (double) targetVectors[i].Y();
    }
    m_cameraToHub = Translation2d(units::meter_t{xSum / targetVectors.size() + kHubRadius}, units::meter_t{ySum / targetVectors.size()});

    // Iterate to find optimal center
    double shiftDist = kHubRadius / 2.0;
    double minResidual = calcResidual(kHubRadius, targetVectors, m_cameraToHub);
    while (true) {
        vector<frc::Translation2d> translations;
        translations.push_back(Translation2d(units::meter_t{shiftDist}, units::meter_t{0.0}));
        translations.push_back(Translation2d(units::meter_t{-shiftDist}, units::meter_t{0.0}));
        translations.push_back(Translation2d(units::meter_t{0.0}, units::meter_t{shiftDist}));
        translations.push_back(Translation2d(units::meter_t{0.0}, units::meter_t{-shiftDist}));
        frc::Translation2d bestPoint = m_cameraToHub;
        bool centerIsBest = true;

        // Check all adjacent positions
        for (int i = 0; i < translations.size(); i++) 
        {
            double residual =
                calcResidual(kHubRadius, targetVectors, m_cameraToHub + (translations[i]));
            if (residual < minResidual) {
                bestPoint = m_cameraToHub + (translations[i]);
                minResidual = residual;
                centerIsBest = false;
                break;
            }
        }
        double precision = 0.01;
        // Decrease shift, exit, or continue
        if (centerIsBest) {
            shiftDist /= 2.0;
            if (shiftDist < precision) {
                return true;
            }
        } else {
            m_cameraToHub = bestPoint;
        }
    }
}

double VisionSubsystem::calcResidual(double radius, vector<frc::Translation2d> points, frc::Translation2d center)
{
    double residual = 0.0;
    for (int i = 0; i < (int)points.size(); i++) {
      double diff = (double) (points[i].Distance(center) - units::meter_t{radius});
      residual += diff * diff;
    }
    return residual;
}

double VisionSubsystem::GetHubAngle()
{
    return atan2(double(m_cameraToHub.X()), double(m_cameraToHub.Y()));
}

double VisionSubsystem::GetHubDistance()
{
    return (double) m_cameraToHub.Norm();
}