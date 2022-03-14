// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousDefault extends SequentialCommandGroup {
  /** Creates a new AutonomousDefault. */
  public AutonomousDefault(DriveSubsystem m_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousDistance(m_robotDrive, 0.2, 0, 5),
      new AutonomousDistance(m_robotDrive, -0.2, 0, 5),
      new AutonomousStop(m_robotDrive));
  }
}
