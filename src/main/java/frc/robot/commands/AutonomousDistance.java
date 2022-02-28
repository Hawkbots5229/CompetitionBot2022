// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDistance extends CommandBase {
  private final DriveSubsystem m_robotDrive;
  private final double m_distance;
  private final double m_speed;

  /**
   * Creates a new AutonomousDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param distance The distance in meters the robot will drive
   * @param robotDrive The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(DriveSubsystem robotDrive, double speed, double distance) {
    this.m_robotDrive = robotDrive;
    this.m_distance = distance;
    this.m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
