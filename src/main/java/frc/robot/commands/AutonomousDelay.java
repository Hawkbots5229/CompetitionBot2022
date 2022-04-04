// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;

public class AutonomousDelay extends CommandBase {
  private final Timer tmr = new Timer();
  /** Creates a new AutonomousDelay. */
  public AutonomousDelay(double delay) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(delay);
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
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tmr.get() > delay;
  }
}
