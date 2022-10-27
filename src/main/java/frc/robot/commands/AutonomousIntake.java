// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorWheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousIntake extends CommandBase {
  private final IntakeSubsystem m_ballIntake;
  private final ElevatorWheelSubsystem m_wheelIntake;
  private final double intakeSpeed;
  private final double elevateWheelSpeed;
  //private final Timer tmr = new Timer();
  /** Creates a new AutonomousIntake. */
  public AutonomousIntake(IntakeSubsystem m_ballIntake, double intakeSpeed, ElevatorWheelSubsystem m_wheelIntake, double elevateWheelSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ballIntake = m_ballIntake;
    this.intakeSpeed = intakeSpeed;
    this.m_wheelIntake = m_wheelIntake;
    this.elevateWheelSpeed = elevateWheelSpeed;
    addRequirements(m_ballIntake);
    addRequirements(m_wheelIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //tmr.reset();
    //tmr.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ballIntake.setTargetOutput(intakeSpeed);
    m_wheelIntake.setTargetOutput(elevateWheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //tmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
