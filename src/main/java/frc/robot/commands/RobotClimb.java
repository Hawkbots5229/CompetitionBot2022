// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotClimb extends CommandBase {

  private final ClimberSubsystem m_robotClimber;
  private final double climbSpeed;

  /** Creates a new RobotCLimb. */
  public RobotClimb(ClimberSubsystem m_robotClimber, double climbSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_robotClimber = m_robotClimber;
    this.climbSpeed = climbSpeed;
    addRequirements(m_robotClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("ClimbSpeed: " + climbSpeed);
    m_robotClimber.setTargetOutput(climbSpeed);
    //m_robotClimber.setTargetVelocity(climbSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotClimber.setTargetOutput(0);
    //m_robotClimber.setTargetVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
