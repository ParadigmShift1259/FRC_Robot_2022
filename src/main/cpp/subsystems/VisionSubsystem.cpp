
#include "subsystems/VisionSubsystem.h"
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
    kCameraHeight = units::inch_t{37};
    kCurrTargetHeight = units::inch_t{8*12 + 8};
    kCameraPitch = units::degree_t{18.8};
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
    frc::Translation2d center;
    if (m_validTarget)
    {
        auto targets = result.GetTargets();

        //Get List of Points
        //if(!m_allPoints.empty())
        
        //Gets Vectors for each target
        for(int i = 0; i < targets.size(); i++)
        {
            kTargetPitch = units::degree_t{targets[i].GetPitch()};
            units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
                kCameraHeight, kCurrTargetHeight, kCameraPitch, kTargetPitch);
            targetVectors.push_back(photonlib::PhotonUtils::EstimateCameraToTargetTranslation(range, frc::Rotation2d(units::degree_t{-targets[i].GetYaw()})));
        }
        
        //Gets all Centers of Targets
        m_centerPoints.clear();
        for(int i =0; i< targets.size(); i++)
        {
            double totalX = 0, totalY = 0;
            auto corners = targets[i].GetCorners();
            for(int j = 0; j < 4; j++)
                {
                    totalX += corners[j].first;
                    totalY += corners[j].second;
                }
            double meanX = totalX/4;
            double meanY = totalY/4;

           m_centerPoints.push_back(std::pair<double, double> (meanX, meanY));
        }
        double radius = 0.601;
        double xSum = 0.0;
        double ySum = 0.0;
        for (int i = 0; i < targetVectors.size(); i++) 
        {
            xSum += (double) targetVectors[i].X();
            ySum += (double) targetVectors[i].Y();
        }
        center = Translation2d(units::meter_t{xSum / targetVectors.size() + radius}, units::meter_t{ySum / targetVectors.size()});

        // Iterate to find optimal center
        double shiftDist = radius / 2.0;
        double minResidual = calcResidual(radius, targetVectors, center);
        while (true) {
            vector<frc::Translation2d> translations;
            translations.push_back(Translation2d(units::meter_t{shiftDist}, units::meter_t{0.0}));
            translations.push_back(Translation2d(units::meter_t{-shiftDist}, units::meter_t{0.0}));
            translations.push_back(Translation2d(units::meter_t{0.0}, units::meter_t{shiftDist}));
            translations.push_back(Translation2d(units::meter_t{0.0}, units::meter_t{-shiftDist}));
            frc::Translation2d bestPoint = center;
            bool centerIsBest = true;

            // Check all adjacent positions
            for (int i = 0; i < translations.size(); i++) 
            {
                double residual =
                    calcResidual(radius, targetVectors, center + (translations[i]));
                if (residual < minResidual) {
                    bestPoint = center + (translations[i]);
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
                    break;
                }
            } else {
                center = bestPoint;
            }
        }
    }

    frc::Pose2d centerPose = Pose2d(units::inch_t{324}, units::inch_t{162}, frc::Rotation2d{units::degree_t{0}});
    units::radian_t angleToTarget = units::radian_t(atan2(double(center.X()), double(center.Y())));
    frc::Pose2d robotPose = photonlib::PhotonUtils::EstimateFieldToCamera(frc::Transform2d(center, angleToTarget), centerPose);

    if(willPrint)
        {
        if (!m_validTarget)
            std::cout << "PhotonCam Has No Targets!" << std::endl;
        else
            {
                std::cout << "Center: (" << (double)center.X() << "," << (double)center.Y() << "). ";
                std::cout << "Target Vectors: ";
                for(int i = 0; i < targetVectors.size(); i++) {
                    //std::cout << "(" << m_centerPoints[i].first << "," << m_centerPoints[i].second << "). ";
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

double VisionSubsystem::calcResidual(double radius, vector<frc::Translation2d> points, frc::Translation2d center)
{
    double residual = 0.0;
    for (int i = 0; i < points.size(); i++) {
      double diff = (double) (points[i].Distance(center) - units::meter_t{radius});
      residual += diff * diff;
    }
    return residual;
}