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
public class Autonomous2Ball extends SequentialCommandGroup {

  /** Creates a new AutonomousDefault. */
  public Autonomous2Ball(DriveSubsystem m_robotDrive, ShooterSubsystem m_robotShoot, ElevatorSubsystem m_robotElevate, IntakeSubsystem m_robotIntake, ElevatorWheelSubsystem m_robotWheelElevate, IntakeHeightSubsystem m_adjustIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // extends intake
      new AdjustIntakeHeight(m_adjustIntake, IntakeConstants.kAdjustIntakeOutput),
      // turns on intake
      new AutonomousIntake(m_robotIntake, IntakeConstants.kIntakeOutput, m_robotWheelElevate, ElevatorConstants.kElevatorMotor3Output),
      // drives to second ball and intakes it
      new AutonomousDistance(m_robotDrive, 0.2, 0, 1.0),
      // turns -10 degrees to aim at target
      new AutonomousRotate(m_robotDrive, 0.2, -10),
      // powers wheel until at target speed and then shoots balls 1 and 2 at high target for 2 seconds
      new AutonomousShootBall(m_robotShoot, ShooterConstants.kHighShooterVelocity, m_robotElevate, ElevatorConstants.kElevatorOutput + 0.1), 
      // turns off intake and elevator wheel
      new AutonomousIntake(m_robotIntake, 0, m_robotWheelElevate, 0),
      // intakes intake
      new AdjustIntakeHeight(m_adjustIntake, -IntakeConstants.kAdjustIntakeOutput));
  }
}
