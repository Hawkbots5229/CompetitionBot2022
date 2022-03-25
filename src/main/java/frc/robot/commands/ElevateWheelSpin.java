// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorWheelSubsystem;

public class ElevateWheelSpin extends CommandBase {

  private final ElevatorWheelSubsystem m_wheelSpin;
  private final double wheelSpeed;

  /** Creates a new ElevateWheelSpin. */
  public ElevateWheelSpin(ElevatorWheelSubsystem m_wheelSpin, double wheelSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_wheelSpin = m_wheelSpin;
    this.wheelSpeed = wheelSpeed;
    addRequirements(m_wheelSpin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wheelSpin.setTargetOutput(wheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wheelSpin.setTargetOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
