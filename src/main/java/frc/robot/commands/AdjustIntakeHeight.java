// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeHeightSubsystem; 

public class AdjustIntakeHeight extends CommandBase {

  private final IntakeHeightSubsystem m_intakeHeight;
  private final double intakeAdjustSpeed;
  
  /** Creates a new AdjustIntakeHeight. */
  public AdjustIntakeHeight(IntakeHeightSubsystem m_intakeHeight, double intakeAdjustSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeHeight = m_intakeHeight;
    this.intakeAdjustSpeed = intakeAdjustSpeed;
    addRequirements(m_intakeHeight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_intakeHeight.resetIntakeEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_intakeHeight.setTargetOutput(intakeAdjustSpeed);
    m_intakeHeight.setTargetPosition(IntakeConstants.kIntakeHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_intakeHeight.setTargetOutput(0); 
    m_intakeHeight.setTargetPosition(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if(intakeAdjustSpeed<0) {
      return (m_intakeHeight.getIntakeHeight() <= 0);
    } else {
      return (m_intakeHeight.getIntakeHeight() >= IntakeConstants.kIntakeHeight);
    }
    */
    return false;
  }
}
