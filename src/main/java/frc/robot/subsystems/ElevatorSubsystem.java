// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_elevatorMotor1 = 
    new WPI_TalonSRX(ElevatorConstants.kElevatorPort1);

  private final TalonSRX m_elevatorMotor2 = 
    new TalonSRX(ElevatorConstants.kElevatorPort2);
  
  private final TalonSRX m_elevatorMotor3 = 
    new TalonSRX(ElevatorConstants.kElevatorPort3);

  /** Creates a new elevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorMotor1.setInverted(ElevatorConstants.kElevatorMotor1Inverted);
    m_elevatorMotor2.setInverted(ElevatorConstants.kElevatorMotor2Inverted);
    m_elevatorMotor3.setInverted(ElevatorConstants.kElevatorMotor3Inverted);
  }

  public void setTargetOutput(double output) {
    m_elevatorMotor1.set(ControlMode.PercentOutput, output);
    m_elevatorMotor2.set(ControlMode.PercentOutput, output);
    m_elevatorMotor3.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
