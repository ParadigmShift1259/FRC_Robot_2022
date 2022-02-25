/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include <frc2/command/button/NetworkButton.h>
#include "AutoPaths.h"

RobotContainer::RobotContainer()
    : m_gyro()
    , m_drive(&m_gyro)
    , m_vision()
    , m_flywheel()
{
    m_fieldRelative = false;

    ConfigureButtonBindings();
    SetDefaultCommands();

    m_chooser.SetDefaultOption("Example 1", AutoPath::kEx1);
    m_chooser.AddOption("Example 2", AutoPath::kEx2);
    m_chooser.AddOption("Example 3", AutoPath::kEx3);
    m_chooser.AddOption("Example 4", AutoPath::kEx4);
    frc::SmartDashboard::PutData("Auto Path", &m_chooser);
}

void RobotContainer::Periodic() {}

void RobotContainer::SetDefaultCommands()
{
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            // up is xbox joystick y pos
            // left is xbox joystick x pos
            /// X and Y are deadzoned twice - once individually with very small values, then another with a pinwheel deadzone
            auto xInput = Util::Deadzone(m_primaryController.GetLeftY() * -1.0, OIConstants::kDeadzoneX);
            auto yInput = Util::Deadzone(m_primaryController.GetLeftX() * -1.0, OIConstants::kDeadzoneY);
            if (Util::Deadzone(sqrt(pow(xInput, 2) + pow(yInput, 2)), OIConstants::kDeadzoneXY) == 0) {
                xInput = 0;
                yInput = 0;
            }

            auto rotInput = Util::Deadzone(m_primaryController.GetRightX() * -1.0, OIConstants::kDeadzoneRot);

            m_drive.Drive(units::meters_per_second_t(xInput * AutoConstants::kMaxSpeed),
                            units::meters_per_second_t(yInput * AutoConstants::kMaxSpeed),
                            units::angular_velocity::radians_per_second_t(rotInput * DriveConstants::kDriveAngularSpeed.to<double>()),
                            m_fieldRelative);
        },
        {&m_drive}
    ));
    // m_flywheel.SetDefaultCommand(
    //     frc2::RunCommand(
    //         [this] {
    //             m_flywheel.SetRPM(FlywheelConstants::kIdleRPM);
    //         }, {&m_flywheel}
    //     )
    // );
}

void RobotContainer::ConfigureButtonBindings()
{
    // Primary
    // Triggers field relative driving
    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kLeftBumper).WhenPressed(
        frc2::InstantCommand(    
            [this] { m_fieldRelative = true; },
            {}
        )
    );

    frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kLeftBumper).WhenReleased(
        frc2::InstantCommand(    
            [this] { m_fieldRelative = false; },
            {}
        )
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kY).WhenPressed(
        Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler,
             &m_turretready, &m_firing, &m_finished, 100.0)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kX).WhenPressed(
        Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler,
             &m_turretready, &m_firing, &m_finished, 200.0)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kRightBumper).WhenPressed(
        Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler,
             &m_turretready, &m_firing, &m_finished, 0.0)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kA).WhenHeld(
        CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed)   
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kLeftBumper).WhenHeld(
        CyclerAgitation(&m_cycler, CyclerConstants::kTurnTableSpeedHigher)   
    );

    // frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kBumperRight).WhenPressed(
    //     frc2::InstantCommand([this] { m_turret.ResetPosition(); }, { &m_turret} )
    // );

    // frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kA).WhenReleased(
    //     CyclerPrepare(&m_cycler, true).WithTimeout(CyclerConstants::kMaxCyclerTime)
    // );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kB).WhenHeld(
        IntakeRelease(&m_intake)
    );

    frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kBack).WhenHeld(
        Unjam(&m_cycler, &m_intake)
    );    

    // frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kA).WhenPressed(
    //     frc2::InstantCommand(    
    //         [this] { 
    //             m_flywheel.SetRPM(m_flywheel.GetRPM() + 100.0);
    //             printf("a");
    //          },
    //         {&m_flywheel}
    //     )
    // );

    // frc2::JoystickButton(&m_primaryController, (int)frc::XboxController::Button::kB).WhenPressed(
    //     frc2::InstantCommand(    
    //         [this] { 
    //             m_flywheel.SetRPM(m_flywheel.GetRPM() - 100.0);
    //             printf("b");
    //          },
    //         {&m_flywheel}
    //     )
    // );

    // Secondary
    // Ex: Triggers Fire sequence
    // frc2::JoystickButton(&m_secondaryController, (int)frc::XboxController::Button::kY).WhenPressed(
    //     Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision,
    //          &m_turretready, &m_firing, &m_finished)
    // );

}

void RobotContainer::ZeroDrive()
{
    m_drive.Drive(units::meters_per_second_t(0.0),
                units::meters_per_second_t(0.0),
                units::radians_per_second_t(0.0), false);
}

frc2::Command *RobotContainer::GetAutonomousCommand(AutoPath path)
{
    switch(path)
    {
        case kEx1:
            return new frc2::SequentialCommandGroup(
                // Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 2.0),
                frc2::ParallelRaceGroup(
                    // CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed),
                    std::move(GetSwerveCommand(left3, sizeof(left3) / sizeof(left3[0]), true))
                ),
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    }, {}
                )
                // Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        case kEx2:
            return new frc2::SequentialCommandGroup(
                std::move(GetSwerveCommand(mid0, sizeof(mid0) / sizeof(mid0[0]), true)),
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    }, {}
                )
                // Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        case kEx3:
            return new frc2::SequentialCommandGroup(
                // Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 1.8),
                frc2::ParallelRaceGroup(
                    // CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed),
                    std::move(GetSwerveCommand(mid5, sizeof(mid5) / sizeof(mid5[0]), true))
                ),
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    }, {}
                )
                // Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        case kEx4:
            return new frc2::SequentialCommandGroup(
                frc2::ParallelRaceGroup(
                    // CyclerIntakeAgitation(&m_intake, &m_cycler, CyclerConstants::kTurnTableSpeed),
                    std::move(GetSwerveCommand(right2, sizeof(right2) / sizeof(right2[0]), true))
                ),
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    }, {}
                )
                // Fire(&m_secondaryController, &m_flywheel, &m_turret, &m_hood, &m_intake, &m_cycler, &m_vision, &m_turretready, &m_firing, &m_finished, 8.0)
            );

        default:
             return new frc2::SequentialCommandGroup(
                frc2::InstantCommand(
                    [this]() {
                        ZeroDrive();
                    }, {}
                )
            );
    }
}

frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> RobotContainer::GetSwerveCommand(double path[][6],  int length, bool primaryPath)
{
    frc::Trajectory exampleTrajectory = convertArrayToTrajectory(path, length);

    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kPThetaController, 0, AutoConstants::kDThetaController,
        AutoConstants::kThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi), units::radian_t(wpi::numbers::pi));

    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> swerveControllerCommand(
        exampleTrajectory,                                                      // frc::Trajectory
        [this]() { return m_drive.GetPose(); },                                 // std::function<frc::Pose2d()>
        m_drive.kDriveKinematics,                                               // frc::SwerveDriveKinematics<NumModules>
        frc2::PIDController(AutoConstants::kPXController, 0, AutoConstants::kDXController),                // frc2::PIDController
        frc2::PIDController(AutoConstants::kPYController, 0, AutoConstants::kDYController),                // frc2::PIDController
        thetaController,                                                        // frc::ProfiledPIDController<units::radians>
        [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
        {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
    );

    // Reset odometry to the starting pose of the trajectory
    if(primaryPath)
        m_drive.ResetOdometry(exampleTrajectory.InitialPose());

    return swerveControllerCommand;
}

// t, v, a, X, Y, H
frc::Trajectory RobotContainer::convertArrayToTrajectory(double a[][6], int length)
{
    std::vector<frc::Trajectory::State> states;
    printf("Converting array...");
    for (int i = 0; i < length; i++)
    {
        printf("looping through timestamps...");
        states.push_back({
            a[i][0] * 1_s, a[i][1] * 1_mps, a[i][2] * 1_mps_sq, 
            frc::Pose2d(a[i][3] * 1_m, a[i][4] * -1.0 * 1_m, a[i][5] * -1.0 * 1_deg), curvature_t(0)
        });
    }
    printf("Finished looping, returning Trajectory");
    return frc::Trajectory(states);
}

void RobotContainer::PrintTrajectory(frc::Trajectory& trajectory)
{
    for (auto &state:trajectory.States())
    {
        double time = state.t.to<double>();
        double x = state.pose.X().to<double>();
        double y = state.pose.Y().to<double>();
        printf("%.3f, %.3f, %.3f\n", time, x, y);
    }
}
