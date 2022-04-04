// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBall extends CommandBase {

  private final ShooterSubsystem m_ballShooter;
  private final double shooterSpeed;
  /** Creates a new ShootBall. */
  public ShootBall(ShooterSubsystem m_ballShooter, double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ballShooter = m_ballShooter;
    this.shooterSpeed = shooterSpeed;
    addRequirements(m_ballShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ballShooter.setTargetVelocity(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballShooter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
