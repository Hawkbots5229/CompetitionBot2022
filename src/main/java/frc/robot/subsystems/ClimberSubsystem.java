// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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

    m_climberLeftFrontMotor.configFactoryDefault();
    m_climberLeftFrontMotor.configFactoryDefault();
    m_climberRightFrontMotor.configFactoryDefault();
    m_climberRightRearMotor.configFactoryDefault();

    m_climberLeftFrontMotor.configNeutralDeadband(0.001); 
    m_climberLeftFrontMotor.configNeutralDeadband(0.001);
    m_climberRightFrontMotor.configNeutralDeadband(0.001);
    m_climberRightRearMotor.configNeutralDeadband(0.001);

    m_climberLeftFrontMotor.setInverted(ClimberConstants.kClimberLeftFrontInverted);
    m_climberLeftRearMotor.setInverted(ClimberConstants.kClimberLeftRearInverted);
    m_climberRightFrontMotor.setInverted(ClimberConstants.kClimberRightFrontInverted);
    m_climberRightRearMotor.setInverted(ClimberConstants.kClimberRightRearInverted);
    
    m_climberLeftFrontMotor.setNeutralMode(ClimberConstants.kClimberNeutralMode);
    m_climberLeftRearMotor.setNeutralMode(ClimberConstants.kClimberNeutralMode);
    m_climberRightFrontMotor.setNeutralMode(ClimberConstants.kClimberNeutralMode);
    m_climberRightRearMotor.setNeutralMode(ClimberConstants.kClimberNeutralMode);
    
    m_climberLeftRearMotor.follow(m_climberLeftFrontMotor);
    m_climberRightFrontMotor.follow(m_climberLeftFrontMotor);
    m_climberRightRearMotor.follow(m_climberLeftFrontMotor);

    /* Config sensor used for Primary PID [Velocity] */
    m_climberLeftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
      ClimberConstants.kPIDLoopIdx, ClimberConstants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    m_climberLeftFrontMotor.configNominalOutputForward(0, ClimberConstants.kTimeoutMs);
		m_climberLeftFrontMotor.configNominalOutputReverse(0, ClimberConstants.kTimeoutMs);
		m_climberLeftFrontMotor.configPeakOutputForward(1, ClimberConstants.kTimeoutMs);
		m_climberLeftFrontMotor.configPeakOutputReverse(-1, ClimberConstants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
		m_climberLeftFrontMotor.config_kF(ClimberConstants.kPIDLoopIdx, ClimberConstants.kF, ClimberConstants.kTimeoutMs);
		m_climberLeftFrontMotor.config_kP(ClimberConstants.kPIDLoopIdx, ClimberConstants.kP, ClimberConstants.kTimeoutMs);
		m_climberLeftFrontMotor.config_kI(ClimberConstants.kPIDLoopIdx, ClimberConstants.kI, ClimberConstants.kTimeoutMs);
		m_climberLeftFrontMotor.config_kD(ClimberConstants.kPIDLoopIdx, ClimberConstants.kD, ClimberConstants.kTimeoutMs);
		
  }

  public void setTargetOutput(double output) {
    m_climberLeftFrontMotor.set(ControlMode.PercentOutput, output);
  }

  // velCCW and velCW are percent max velocity [-1:1]
  public void setTargetVelocity(double velCCW, double velCW) {

    double armRevPerMin = (velCCW + velCW) * ClimberConstants.kMaxSpeed;
    //System.out.println("ArmRevPerMin: " + armRevPerMin);
    
    //double targetVelocity_RevPer100ms = ((velCCW + velCW) * ClimberConstants.kMaxSpeed * ClimberConstants.kGearRatio) / 600.0;
    double targetVelocity_RevPer100ms = armRevPerMin * ClimberConstants.kGearRatio * 600; //MotRevPer100MilliSec

    //System.out.println("TarVel_RevPer100ms: " + targetVelocity_RevPer100ms);

    m_climberLeftFrontMotor.set(TalonFXControlMode.Velocity, targetVelocity_RevPer100ms);
  }
  
  public double getClimberOutput() {
    return ((m_climberLeftFrontMotor.getMotorOutputPercent() + m_climberLeftRearMotor.getMotorOutputPercent() + m_climberRightFrontMotor.getMotorOutputPercent() + m_climberRightRearMotor.getMotorOutputPercent())/4);
  }

  public double getClimberVelocity() {
    
    // Convert MotRevPer100MilliSec to ArmRevPerMin
    return ((m_climberLeftFrontMotor.getSelectedSensorVelocity() * 100) / (ClimberConstants.kGearRatio * 600));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Output", getClimberOutput());
    SmartDashboard.putNumber("Climber Velocity", getClimberVelocity());

  }
}
