// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonSRX m_intakeMotor = 
    new TalonSRX(IntakeConstants.kIntakePort);

  /** Creates a new intakeSubsytem. */
  public IntakeSubsystem() {
    m_intakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);
  }

  public void setTargetOutput(double output) {
    m_intakeMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
