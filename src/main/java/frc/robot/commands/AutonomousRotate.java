// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousRotate extends CommandBase {
  private final DriveSubsystem m_robotDrive;
  private final double m_degrees;
  private final double m_rotSpeed;

  /**
   * Creates a new AutonomousRotate. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param rotSpeed The angular speed of the robot. Clockwise is positive.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param robotDrive The drivetrain subsystem on which this command will run
   */
  public AutonomousRotate(DriveSubsystem robotDrive, double rotSpeed, double degrees) {
    this.m_robotDrive = robotDrive;
    this.m_degrees = degrees;
    this.m_rotSpeed = rotSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.drivePID(0.0, 0.0, m_rotSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_robotDrive.getAngle()) >= m_degrees;
  }
}
