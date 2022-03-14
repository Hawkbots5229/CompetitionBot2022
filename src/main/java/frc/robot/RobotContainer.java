// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ShootBall;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutonomousDefault;
import frc.robot.commands.ElevateBall;
import frc.robot.commands.IntakeBall;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_robotShooter = new ShooterSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final ElevatorSubsystem m_robotElevate = new ElevatorSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                  -m_driverController.getLeftY(),
                    m_driverController.getLeftX(),                   
                    m_driverController.getRightX(),
                    false),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));

    //Dump when x is pressed
    new JoystickButton(m_driverController, Button.kX.value)
        .toggleWhenPressed(new ShootBall(m_robotShooter, ShooterConstants.kLowShooterOutput));
    
    //Shoot at normal speed when A is pressed
    new JoystickButton(m_driverController, Button.kA.value)
        .toggleWhenPressed(new ShootBall(m_robotShooter, ShooterConstants.kHighShooterOutput));
    
    new JoystickButton(m_driverController, Button.kB.value)
        .toggleWhenPressed(new IntakeBall(m_robotIntake, IntakeConstants.kIntakeOutput));

    new POVButton(m_driverController, OIConstants.kUpDPad)
        .toggleWhenPressed(new ElevateBall(m_robotElevate, ElevatorConstants.kElevatorOutput));

    new POVButton(m_driverController, OIConstants.kDownDPad)
        .toggleWhenPressed(new ElevateBall(m_robotElevate, -ElevatorConstants.kElevatorOutput));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Basic Auto", new AutonomousDefault(m_robotDrive));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();    
  }
}
