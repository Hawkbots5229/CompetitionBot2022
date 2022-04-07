// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeHeightSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_intakeHeightMotor = 
    new WPI_TalonSRX(IntakeConstants.kIntake2Port);

  /** Creates a new intakeHeightSubsytem. */
  public IntakeHeightSubsystem() {
    m_intakeHeightMotor.configFactoryDefault();
    m_intakeHeightMotor.configNeutralDeadband(0.001); 
    m_intakeHeightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      IntakeConstants.kPIDLoopIdx, IntakeConstants.kTimeoutMs);
    m_intakeHeightMotor.setSensorPhase(IntakeConstants.kSensorPhase);
    m_intakeHeightMotor.setInverted(IntakeConstants.kIntakeHeightMotorInverted);
    m_intakeHeightMotor.setNeutralMode(IntakeConstants.kIntakeHeightMotorNeutralMode);
    m_intakeHeightMotor.configNominalOutputForward(0, IntakeConstants.kTimeoutMs);
		m_intakeHeightMotor.configNominalOutputReverse(0, IntakeConstants.kTimeoutMs);
		m_intakeHeightMotor.configPeakOutputForward(1, IntakeConstants.kTimeoutMs);
		m_intakeHeightMotor.configPeakOutputReverse(-1, IntakeConstants.kTimeoutMs);
    m_intakeHeightMotor.configAllowableClosedloopError(0, IntakeConstants.kPIDLoopIdx, IntakeConstants.kTimeoutMs);
    m_intakeHeightMotor.config_kF(IntakeConstants.kPIDLoopIdx, IntakeConstants.kF, IntakeConstants.kTimeoutMs);
		m_intakeHeightMotor.config_kP(IntakeConstants.kPIDLoopIdx, IntakeConstants.kP, IntakeConstants.kTimeoutMs);
		m_intakeHeightMotor.config_kI(IntakeConstants.kPIDLoopIdx, IntakeConstants.kI, IntakeConstants.kTimeoutMs);
		m_intakeHeightMotor.config_kD(IntakeConstants.kPIDLoopIdx, IntakeConstants.kD, IntakeConstants.kTimeoutMs);
    /**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		//int absolutePosition = m_intakeHeightMotor.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		//absolutePosition &= 0xFFF;
		//if (IntakeConstants.kSensorPhase) { absolutePosition *= -1; }
		//if (IntakeConstants.kIntakeHeightMotorInverted) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		m_intakeHeightMotor.setSelectedSensorPosition(0, IntakeConstants.kPIDLoopIdx, IntakeConstants.kTimeoutMs);
  }

  public void setTargetOutput(double output) {
    m_intakeHeightMotor.set(ControlMode.PercentOutput, output);

  }

  public void setTargetPosition(double targetPos) {
    m_intakeHeightMotor.set(ControlMode.Position, targetPos);
  }
  
  public void resetIntakeEncoder() {
    m_intakeHeightMotor.setSelectedSensorPosition(0);
  }
  
  public double getIntakeHeight() {
    
    return (m_intakeHeightMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Hight", getIntakeHeight());
  }
}
