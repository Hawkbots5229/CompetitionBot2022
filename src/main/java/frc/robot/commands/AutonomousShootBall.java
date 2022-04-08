// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousShootBall extends CommandBase {

  private final ShooterSubsystem m_ballShooter;
  private final double shooterSpeed;
  private final ElevatorSubsystem m_ballElevate;
  private final double elevateSpeed;
  private final Timer tmr = new Timer();
  
  /** Creates a new ShootBall. */
  public AutonomousShootBall(ShooterSubsystem m_ballShooter, double shooterSpeed, ElevatorSubsystem m_ballElevate, double elevateSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ballShooter = m_ballShooter;
    this.shooterSpeed = shooterSpeed;
    this.m_ballElevate = m_ballElevate;
    this.elevateSpeed = elevateSpeed;
    addRequirements(m_ballElevate);
    addRequirements(m_ballShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tmr.reset();
    tmr.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ballShooter.setTargetVelocity(shooterSpeed);
    if(m_ballShooter.getShooterVelocity() > shooterSpeed){
      m_ballElevate.setTargetOutput(elevateSpeed);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballShooter.setTargetOutput(0);
    m_ballElevate.setTargetOutput(0);
    tmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tmr.get() > AutoConstants.kShooterAutonDelay;
  }
}
