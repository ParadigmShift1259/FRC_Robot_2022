/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include <frc2/command/button/NetworkButton.h>
#include <frc2/command/WaitCommand.h>

RobotContainer::RobotContainer()
    : m_gyro()
    , m_drive(&m_gyro)
    , m_vision(&m_gyro, m_turret, m_hood, *this)
    , m_flywheel()
{
    m_fieldRelative = false;

    ConfigureButtonBindings();
    SetDefaultCommands();

    m_chooser.SetDefaultOption("Path 1", EAutoPath::kEx1);
    m_chooser.AddOption("Path 2", EAutoPath::kEx2);
    m_chooser.AddOption("Path 3", EAutoPath::kEx3);
    m_chooser.AddOption("Path 4", EAutoPath::kEx4);
    m_chooser.AddOption("Path 5", EAutoPath::kEx5);
    frc::SmartDashboard::PutData("Auto Path", &m_chooser);

    SmartDashboard::PutNumber("servo override", 0.0);

    SmartDashboard::PutNumber("fudge", 0.0);
    SmartDashboard::PutBoolean("UseFudgeFactor", false);
    SmartDashboard::PutBoolean("UseLut", false);
    SmartDashboard::PutBoolean("SupressFlywheel", false);
    SmartDashboard::PutBoolean("LowSpeedDriveing", m_bLowSpeedDriving);
}

void RobotContainer::Periodic()
{
    SmartDashboard::PutNumber("Gyro", m_gyro.GetHeading());
    // SmartDashboard::PutData("DriveSS", &m_drive);
    // SmartDashboard::PutData("FlywheelSS", &m_flywheel);
    // SmartDashboard::PutData("HoodSS", &m_hood);
    // SmartDashboard::PutData("IntakeSS", &m_intake);
    // SmartDashboard::PutData("TransferSS", &m_transfer);
    // SmartDashboard::PutData("TurretSS", &m_turret);
    // SmartDashboard::PutData("VisionSS", &m_vision);
    SmartDashboard::PutNumber("Hub angle ", m_vision.GetHubAngle());
    m_drive.Periodic();

    // static int direction = 0;
    // if(direction == 1)
    // {
    //     if(m_turret.GetCurrentAngle() > 48)
    //     {
    //         direction = -1;
    //         m_turret.TurnTo(-50);
    //         std::cout << "Reversing" << "\n";
    //     }
    // }
    // else if (direction == -1)
    // {
    //     if(m_turret.GetCurrentAngle() < -48)
    //     {
    //         direction = 1;
    //         m_turret.TurnTo(50);
    //         std::cout << "Going Forward" << "\n";
    //     }
    // }
    // else
    // {
    //     // first time only
    //     direction = 1;
    //     m_turret.TurnTo(60);
    //     std::cout << "Initial" << "\n";
    // }
}

void RobotContainer::SetDefaultCommands()
{
#define USE_DRIVE	// Comment this define out when robot in on top of battery cart
#ifdef USE_DRIVE
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            // up is xbox joystick y pos
            // left is xbox joystick x pos
            /// X and Y are deadzoned twice - once individually with very small values, then another with a pinwheel deadzone

            auto xInput = m_primaryController.GetLeftY() * -1.0;    // The x robot axis is driven from the Y joystick axis
            auto yInput = m_primaryController.GetLeftX() * -1.0;

            xInput = Util::Deadzone(xInput, OIConstants::kDeadzoneX);
            yInput = Util::Deadzone(yInput, OIConstants::kDeadzoneY);
            auto magnitude = sqrt(pow(xInput, 2.0) + pow(yInput, 2.0));
            if (Util::Deadzone(magnitude, OIConstants::kDeadzoneXY) == 0)
            {
                xInput = 0;
                yInput = 0;
            }

            auto rotInput = Util::Deadzone(m_primaryController.GetRightX() * -1.0, OIConstants::kDeadzoneRot);

            m_drive.Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                            units::angular_velocity::radians_per_second_t(rotInput * m_maxRotSpeed.to<double>()),
                            m_fieldRelative);
        },
        {&m_drive}
    ));
#endif
    m_flywheel.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                m_flywheel.SetRPM(FlywheelConstants::kIdleRPM);
            }, {&m_flywheel}
        )
    );

    m_climber.SetDefaultCommand(
        frc2::RunCommand(
            [this] {
                m_climber.Run(0.0);
            }, {&m_climber}
        )
    );    
}

void RobotContainer::ConfigureButtonBindings()
{
    using namespace frc;
    using namespace frc2;
    using xbox = frc::XboxController::Button;

    // Primary
    // Triggers field relative driving
    JoystickButton(&m_primaryController, xbox::kLeftBumper).WhenPressed(&m_setFieldRelative);
    JoystickButton(&m_primaryController, xbox::kLeftBumper).WhenReleased(&m_clearFieldRelative);

    // Toggle slow speed driving for strafe shot
    JoystickButton(&m_primaryController, xbox::kRightBumper).WhenPressed(&m_toggleMaxDriveSpeed);

    JoystickButton(&m_secondaryController, xbox::kY).WhenPressed(

        Fire(&m_flywheel, &m_turret, &m_hood, &m_transfer, m_vision, &m_turretready, &m_firing, &m_finished, [this]() { return GetYvelovity(); } )
    );

    JoystickButton(&m_secondaryController, xbox::kX).WhenPressed(
             InstantCommand(    
             [this] { 
                m_transfer.SetFeeder(.5);
                m_transfer.SetTransfer(0.5);
              },
             {&m_transfer}
         )
    );

     JoystickButton(&m_secondaryController, xbox::kX).WhenReleased(
         InstantCommand(    
             [this] { 
                if (m_dbgSeroTest)
                {
                    auto s = SmartDashboard::GetNumber("servo override", 0.0);
                    m_hood.Set(s);
                }
                else
                {
                    m_transfer.SetFeeder(0.0);
                    m_transfer.SetTransfer(0.0);
                    // m_turret.SetZeroAngle();
                    // m_flywheel.SetRPM(FlywheelConstants::kIdleRPM);
                }
              },
             {&m_transfer}
         )
    );

    JoystickButton(&m_secondaryController, xbox::kLeftBumper).WhenHeld(
         InstantCommand(    
             [this] { 
                m_turret.SetZeroAngle();
              },
             {&m_turret}
         )
    );

 
    JoystickButton(&m_primaryController, xbox::kA).WhenHeld(
         InstantCommand(    
             [this] { 
                 
                 //m_overrideAngle += 10.0;
                m_turret.TurnToRelative(50.0);
              },
             {&m_turret}
         )
    );

    JoystickButton(&m_primaryController, xbox::kY).WhenHeld(
         InstantCommand(    
             [this] { 
                // m_overrideAngle -= 10.0;
                m_turret.TurnToRelative(-50.0);
               },
             {&m_turret}
         )
    );

    // JoystickButton(&m_primaryController, xbox::kX).WhenPressed(&m_zeroHeading);  REMOVED FOR GAME PLAY!
    JoystickButton(&m_primaryController, xbox::kBack).WhileHeld(&m_climb);

    // JoystickButton(&m_secondaryController, xbox::kRightBumper).WhenPressed(
    //     Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_transfer, m_vision,
    //          &m_turretready, &m_firing, &m_finished)
    // );

    JoystickButton(&m_secondaryController, xbox::kA).WhenPressed(IntakeTransfer(*this, TransferConstants::kTransferSpeedIntaking));

    // JoystickButton(&m_secondaryController, xbox::kLeftBumper).WhenHeld(
    //     TransferFirstBall(&m_transfer, TransferConstants::kTransferSpeedIntaking),
    //     TransferSecondBall(&m_transfer, TransferConstants::kTransferSpeedIntaking)
    // );

    // JoystickButton(&m_secondaryController, xbox::kBumperRight).WhenPressed(
    //     InstantCommand([this] { m_turret.ResetPosition(); }, { &m_turret} )
    // );

    // JoystickButton(&m_secondaryController, xbox::kA).WhenReleased(
    //     TransferPrepare(&m_transfer, true).WithTimeout(TransferConstants::kMaxTransferTime)
    // );

    JoystickButton(&m_secondaryController, xbox::kB).WhenHeld(IntakeRelease(*this));
    JoystickButton(&m_secondaryController, xbox::kBack).WhenHeld(Unjam(&m_transfer, &m_intake));    

    // JoystickButton(&m_primaryController, xbox::kA).WhenPressed(
    //     InstantCommand(    
    //         [this] { 
    //             m_flywheel.SetRPM(m_flywheel.GetRPM() + 100.0);
    //             printf("a");
    //          },
    //         {&m_flywheel}
    //     )
    // );

    // JoystickButton(&m_primaryController, xbox::kB).WhenPressed(
    //     InstantCommand(    
    //         [this] { 
    //             m_flywheel.SetRPM(m_flywheel.GetRPM() - 100.0);
    //             printf("b");
    //          },
    //         {&m_flywheel}
    //     )
    // );

    // Secondary
    // Ex: Triggers Fire sequence
    // JoystickButton(&m_secondaryController, xbox::kY).WhenPressed(
    //     Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_transfer, &m_vision,
    //          &m_turretready, &m_firing, &m_finished)
    // );
}

const units::meters_per_second_t zeroMps{0.0};
const units::radians_per_second_t zeroRadsPerSec{0.0};

void RobotContainer::ZeroDrive()
{
    m_drive.Drive(zeroMps, zeroMps, zeroRadsPerSec, false);
}

frc2::Command* RobotContainer::GetAutonomousCommand(EAutoPath path)
{
    switch (path)
    {
        case kEx1:
            return GetAutoPathCmd("ball1", true);

        case kEx2:
            return GetAutoPathCmd("ball2&3", false);

        case kEx3:
            return GetAutoPathCmd("ball4", false);

        case kEx4:
            return new frc2::SequentialCommandGroup
            (
                  std::move(*GetAutoPathCmd("ball1", true))
                , std::move(*GetAutoPathCmd("ball2&3", false))
                , std::move(*GetAutoPathCmd("ball4", false))    // TODO only one ball, will transfer cmd end?
            );

        default:
            return new frc2::InstantCommand([this]() { ZeroDrive(); }, {&m_drive});
    }
}

frc2::SequentialCommandGroup* RobotContainer::GetAutoPathCmd(string pathName, bool primaryPath)
{
    return new frc2::SequentialCommandGroup
    (
        frc2::ParallelCommandGroup
        (
              std::move(IntakeTransfer(*this, TransferConstants::kTransferSpeedIntaking))
            , frc2::SequentialCommandGroup
            (
                  std::move(GetSwerveCommandPath(pathName, primaryPath))
                , frc2::WaitCommand(0.1_s)
                , frc2::InstantCommand([this]() { ZeroDrive(); }, {&m_drive})
            )
        )
        , std::move(Fire( &m_flywheel
                        , &m_turret
                        , &m_hood
                        , &m_transfer
                        , m_vision
                        , &m_turretready
                        , &m_firing
                        , &m_finished
                        , [this]() { return GetYvelovity(); }
                        , TransferConstants::kTimeLaunch))
    );
}

SwerveCtrlCmd RobotContainer::GetSwerveCommandPath(string pathName, bool primaryPath)
{
    PathPlannerTrajectory path = PathPlanner::loadPath(pathName, AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
    // PathPlannerTrajectory path = PathPlanner::loadPath(pathName, 1.0 * 1_mps, 2.0 * 1_mps_sq);

    frc::Trajectory trajectory = convertPathToTrajectory(path);
    PrintTrajectory(trajectory);

    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, AutoConstants::kDThetaController,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi), units::radian_t(wpi::numbers::pi));

    SwerveCtrlCmd swerveControllerCommand(
        trajectory,                                                             // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(AutoConstants::kPXController, 0, AutoConstants::kDXController),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, AutoConstants::kDYController),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );

    // Reset odometry to the starting pose of the trajectory
    if (primaryPath)
        {
        // Init absolute gyro angle isn't required by ResetOdometry() but IS required due to directly reading the gyro elsewhere
        m_gyro.SetHeading((double)trajectory.InitialPose().Rotation().Degrees()); 
        m_drive.ResetOdometry(trajectory.InitialPose());
        }

    return swerveControllerCommand;
}

frc::Trajectory RobotContainer::convertPathToTrajectory(PathPlannerTrajectory path)
{
    std::vector<frc::Trajectory::State> states;
    for (double time = 0; time < path.getTotalTime().to<double>(); time += 0.02)
    {
        PathPlannerTrajectory::PathPlannerState state = path.sample(time * 1_s);
        //printf("time %.3f holorot %.3f\n", state.holonomicRotation.Degrees().to<double>());
        states.push_back({
            time * 1_s,
            state.velocity,
            state.acceleration, 
            frc::Pose2d(
                state.pose.X(),
                state.pose.Y(),
                state.holonomicRotation
            ),
            curvature_t(0)
        });
    }
    return frc::Trajectory(states);
}

void RobotContainer::PrintTrajectory(frc::Trajectory& trajectory)
{
    printf("Time,X,Y,HoloRot\n");
    for (auto &state:trajectory.States())
    {
        double time = state.t.to<double>();
        double x = state.pose.X().to<double>();
        double y = state.pose.Y().to<double>();
        double holoRot = state.pose.Rotation().Degrees().to<double>();
        printf("%.3f,%.3f,%.3f,%.3f\n", time, x, y, holoRot);
    }
}
