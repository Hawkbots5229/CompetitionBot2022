// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevateBall extends CommandBase {

  private final ElevatorSubsystem m_ballElevate;
  private final double elevateSpeed;

  /** Creates a new ElevateBall. */
  public ElevateBall(ElevatorSubsystem m_ballElevate, double elevateSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ballElevate = m_ballElevate;
    this.elevateSpeed = elevateSpeed;
    addRequirements(m_ballElevate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ballElevate.setTargetOutput(elevateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballElevate.setTargetOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
