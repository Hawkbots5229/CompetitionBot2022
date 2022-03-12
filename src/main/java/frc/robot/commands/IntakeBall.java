// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBall extends CommandBase {

  private final IntakeSubsystem m_ballIntake;
  private final double intakeSpeed;


  /** Creates a new IntakeBall. */
  public IntakeBall(IntakeSubsystem m_ballIntake, double intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ballIntake = m_ballIntake;
    this.intakeSpeed = intakeSpeed;
    addRequirements(m_ballIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ballIntake.setTargetOutput(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballIntake.setTargetOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
