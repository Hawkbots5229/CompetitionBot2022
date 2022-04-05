// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_climberLeftMotor = 
    new WPI_TalonFX(ClimberConstants.kClimberLeftPort);

  private final WPI_TalonFX m_climberRightMotor = 
    new WPI_TalonFX(ClimberConstants.kClimberRightPort);

  /** Creates a new climberSubsystem. */
  public ClimberSubsystem() {
    m_climberLeftMotor.setInverted(ClimberConstants.kClimberLeftMotorInverted);
    m_climberRightMotor.setInverted(ClimberConstants.kClimberRightMotorInverted);
  }

  public void setTargetOutput(double output) {
    m_climberLeftMotor.set(ControlMode.PercentOutput, output);
    m_climberRightMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
