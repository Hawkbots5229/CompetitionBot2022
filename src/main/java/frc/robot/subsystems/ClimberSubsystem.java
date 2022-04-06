// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_climberLeftFrontMotor = 
    new WPI_TalonFX(ClimberConstants.kClimberLeftFrontPort);

  private final WPI_TalonFX m_climberLeftRearMotor = 
    new WPI_TalonFX(ClimberConstants.kClimberLeftRearPort);

  private final WPI_TalonFX m_climberRightFrontMotor = 
    new WPI_TalonFX(ClimberConstants.kClimberRightFrontPort);

  private final WPI_TalonFX m_climberRightRearMotor = 
    new WPI_TalonFX(ClimberConstants.kClimberRightRearPort);

  /** Creates a new climberSubsystem. */
  public ClimberSubsystem() {
    m_climberLeftFrontMotor.setInverted(ClimberConstants.kClimberLeftFrontInverted);
    m_climberLeftRearMotor.setInverted(ClimberConstants.kClimberLeftRearInverted);
    m_climberRightFrontMotor.setInverted(ClimberConstants.kClimberRightFrontInverted);
    m_climberRightRearMotor.setInverted(ClimberConstants.kClimberRightRearInverted);
    
    m_climberLeftFrontMotor.setNeutralMode(ClimberConstants.kClimberNeutralMode);
    m_climberLeftRearMotor.setNeutralMode(ClimberConstants.kClimberNeutralMode);
    m_climberRightFrontMotor.setNeutralMode(ClimberConstants.kClimberNeutralMode);
    m_climberRightRearMotor.setNeutralMode(ClimberConstants.kClimberNeutralMode);
    
    m_climberLeftRearMotor.follow(m_climberLeftFrontMotor);
    m_climberRightRearMotor.follow(m_climberRightFrontMotor);
  }

  public void setTargetOutput(double output) {
    m_climberLeftFrontMotor.set(ControlMode.PercentOutput, output);
    m_climberRightFrontMotor.set(ControlMode.PercentOutput, output);
  }

  public void setTargetVelocity(double velocity) {
    m_climberLeftFrontMotor.set(TalonFXControlMode.Velocity, velocity);
    m_climberLeftFrontMotor.set(TalonFXControlMode.Velocity, velocity);
  }
  
  public double getClimberOutput() {
    return ((m_climberLeftFrontMotor.getMotorOutputPercent() + m_climberLeftRearMotor.getMotorOutputPercent() + m_climberRightFrontMotor.getMotorOutputPercent() + m_climberRightRearMotor.getMotorOutputPercent())/4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Output", getClimberOutput());
  }
}
