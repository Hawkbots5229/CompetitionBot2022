// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorWheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeHeightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous4Ball extends SequentialCommandGroup {

  /** Creates a new AutonomousDefault. */
  public Autonomous4Ball(DriveSubsystem m_robotDrive, ShooterSubsystem m_robotShoot, ElevatorSubsystem m_robotElevate, IntakeSubsystem m_robotIntake, ElevatorWheelSubsystem m_robotWheelElevate, IntakeHeightSubsystem m_adjustIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // extends intake
      new AdjustIntakeHeight(m_adjustIntake, IntakeConstants.kIntakeHeight),
      // turns to line up with second ball
      new AutonomousRotate(m_robotDrive, 0.2, 70),
      // drives to line up with second ball
      new AutonomousDistance(m_robotDrive, 0.2, 0, 0.2),
      // turns toward second ball
      new AutonomousRotate(m_robotDrive, 0.2, -90),
      // turns on intake and elevator wheel
      new AutonomousIntake(m_robotIntake, IntakeConstants.kIntakeOutput, m_robotWheelElevate, ElevatorConstants.kElevatorMotor3Output),
      // drives to second ball and intakes it
      new AutonomousDistance(m_robotDrive, 0.2, 0, 1.1),
      // turns 10 degrees to aim at target
      new AutonomousRotate(m_robotDrive, 0.2, 10),
      // powers wheel for 1 second and then shoots balls 1 and 2 at high target for 2 seconds
      new AutonomousShootBall(m_robotShoot, ShooterConstants.kHighShooterVelocity, m_robotElevate, ElevatorConstants.kElevatorOutput + 0.1, m_robotWheelElevate, ElevatorConstants.kElevatorMotor3Output), 
      // turns off intake and elevator wheel
      new AutonomousIntake(m_robotIntake, 0, m_robotWheelElevate, 0),
      // turns to line up with the 3rd ball
      new AutonomousRotate(m_robotDrive, 0.2, 80),
      // drives to line up with third ball
      new AutonomousDistance(m_robotDrive, 0.2, 0, 2.08), 
      // turns towards third ball
      new AutonomousRotate(m_robotDrive, 0.2, -90),
      // turns on intake and elevator wheel
      new AutonomousIntake(m_robotIntake, IntakeConstants.kIntakeOutput, m_robotWheelElevate, ElevatorConstants.kElevatorMotor3Output),
      // drives to third ball and intakes it
      new AutonomousDistance(m_robotDrive, 0.2, 0, 3.55),
      // adds delay for 2 seconds for human player to load fourth ball
      new AutonomousDelay(AutoConstants.kAutonDelay), 
      // turns towards shooting spot
      new AutonomousRotate(m_robotDrive, 0.2, 30),
      // drives back to shooting spot
      new AutonomousDistance(m_robotDrive, 0.2, 0, -4.1),
      // aims at high target
      new AutonomousRotate(m_robotDrive, 0.2, 20),
      // powers wheel for 1 second and then shoots ball 3 and 4 at high target for 2 seconds
      new AutonomousShootBall(m_robotShoot, ShooterConstants.kHighShooterVelocity, m_robotElevate, ElevatorConstants.kElevatorOutput + 0.1, m_robotWheelElevate, ElevatorConstants.kElevatorMotor3Output),
      // turns off intake and elevator wheel
      new AutonomousIntake(m_robotIntake, 0, m_robotWheelElevate, 0),
      // intaks intake height
      new AdjustIntakeHeight(m_adjustIntake, 0));
  }
}
