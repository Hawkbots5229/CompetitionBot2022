// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new shooterSubsystem. */

  //private final TalonSRX m_shooterMotor = 
    //new TalonSRX(ShooterConstants.kShooterPort); // 775 pro

  private final CANSparkMax m_shooterMotor = 
    new CANSparkMax(ShooterConstants.kShooterPort, MotorType.kBrushless); // Neo 550

  private final RelativeEncoder m_shooterMotorEncoder = m_shooterMotor.getEncoder();

  private final SparkMaxPIDController m_shooterMotorVelPIDController = m_shooterMotor.getPIDController();

  public ShooterSubsystem() {
    m_shooterMotor.restoreFactoryDefaults();
    m_shooterMotor.setIdleMode(ShooterConstants.kIdleMode);
    m_shooterMotor.setInverted(ShooterConstants.kShooterMotorInverted);
    m_shooterMotor.setClosedLoopRampRate(ShooterConstants.kClosedLoopRampRate);
    //m_shooterMotor.setSmartCurrentLimit(40, 5, ShooterConstants.kHighShooterVelocity);
    m_shooterMotor.setSecondaryCurrentLimit(50);

    m_shooterMotorVelPIDController.setFF(ShooterConstants.kFVel, ShooterConstants.kVelPidSlot); 
    m_shooterMotorVelPIDController.setP(ShooterConstants.kPVel, ShooterConstants.kVelPidSlot);
    m_shooterMotorVelPIDController.setD(ShooterConstants.kDVel, ShooterConstants.kVelPidSlot);
    m_shooterMotorVelPIDController.setI(ShooterConstants.kIVel, ShooterConstants.kVelPidSlot);

  }

  public void setTargetOutput(double output) {
    //m_shooterMotor.set(ControlMode.PercentOutput, output); // 775 pro
    m_shooterMotor.set(output); // Neo 550
  }

  public void setTargetVelocity(double Velocity) {
    m_shooterMotorVelPIDController.setReference(
      Velocity, 
      CANSparkMax.ControlType.kVelocity, 
      ShooterConstants.kVelPidSlot);
  }

  public double getShooterVelocity() {
    return m_shooterMotorEncoder.getVelocity();
  }

  public void stopMotor(){
    m_shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", m_shooterMotorEncoder.getVelocity());
  }

}
