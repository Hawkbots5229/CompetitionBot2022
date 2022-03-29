// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new shooterSubsystem. */

  //private final TalonSRX m_shooterMotor = 
    //new TalonSRX(ShooterConstants.kShooterPort); // 775 pro

  private final CANSparkMax m_shooterMotor = 
    new CANSparkMax(ShooterConstants.kShooterPort, MotorType.kBrushless); // Neo 550

  public ShooterSubsystem() {
    m_shooterMotor.setInverted(ShooterConstants.kShooterMotorInverted);
  }

  public void setTargetOutput(double output) {
    //m_shooterMotor.set(ControlMode.PercentOutput, output); // 775 pro
    m_shooterMotor.set(output); // Neo 550
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
