// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorWheelSubsystem extends SubsystemBase {
  /** Creates a new ElevatorWheelSubsystem. */

  private final TalonSRX m_wheelMotor = 
    new TalonSRX(ElevatorConstants.kElevatorPort3);

  public ElevatorWheelSubsystem() {
    m_wheelMotor.setInverted(ElevatorConstants.kElevatorMotor3Inverted);
  }

  public void setTargetOutput(double output) {
    m_wheelMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
