// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousStop extends CommandBase {

  private final DriveSubsystem m_robotDrive;

  /**
   * Creates a new AutonomousStop. This command will stop the drive motors.
   * This command will never end. Put it at the end of the autonomous manuever
   * to eliminate the error 'MecanumDrive... Output not updated often enough.'.
   *
   * @param robotDrive The drivetrain subsystem on which this command will run
   */
  public AutonomousStop(DriveSubsystem robotDrive) {

    this.m_robotDrive = robotDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {m_robotDrive.stopMotors();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {m_robotDrive.stopMotors();}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {m_robotDrive.stopMotors();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
