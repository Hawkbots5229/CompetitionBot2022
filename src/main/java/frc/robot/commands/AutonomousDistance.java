// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousDistance extends CommandBase {
  private final DriveSubsystem m_robotDrive;
  private final double m_distance;
  private final double m_ySpeed;
  private final double m_xSpeed;
  private final Timer tmr = new Timer();
  private double tarVel;

  /**
   * Creates a new AutonomousDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param ySpeed Speed of the robot in the y direction (forward).
   * @param xSpeed Speed of the robot in the x direction (sideways).
   * @param distance The distance in meters the robot will drive
   * @param robotDrive The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(DriveSubsystem robotDrive, double ySpeed, double xSpeed, double distance) {
    this.m_robotDrive = robotDrive;
    this.m_distance = distance;
    this.m_ySpeed = ySpeed;
    this.m_xSpeed = xSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.resetEncoders();
    tmr.reset();
    tmr.start();
    tarVel = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tmr.get() < DriveConstants.kClosedLoopRampRate) {
      tarVel = (m_ySpeed/DriveConstants.kClosedLoopRampRate) * tmr.get();
    } else {
      tarVel = m_ySpeed;
    }
    SmartDashboard.putNumber("Target Velocity", tarVel*DriveConstants.kMaxSpeedMetersPerSecond);
    //System.out.println(tarVel*DriveConstants.kMaxSpeedMetersPerSecond);
    m_robotDrive.drivePID(m_ySpeed, m_xSpeed, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.stopMotors();
    tmr.stop();
    tarVel = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_robotDrive.getRobotPosition() > m_distance;
  }
}
