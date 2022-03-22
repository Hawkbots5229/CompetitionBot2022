// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftMotorPort = 2;
    public static final int kRearLeftMotorPort = 4;
    public static final int kFrontRightMotorPort = 3;
    public static final int kRearRightMotorPort = 1;

    public static final boolean kFrontLeftMotorReversed = false;
    public static final boolean kRearLeftMotorReversed = false;
    public static final boolean kFrontRightMotorReversed = true; //true
    public static final boolean kRearRightMotorReversed = true; //true

    public static final boolean kGyroInverted = false; // Clockwise should be a positive angle   

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.5461; // Meters (21.5 inches) 
    // Distance between centers of front and back wheels on robot
    public static final double kWheelBase = 0.51435; // Meters (20.25 inches)
    
    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 40; // Amps
    public static final double kOpenLoopRampRate = 0.7; // Time in seconds to go from 0 to full throttle.
    public static final double kClosedLoopRampRate = 0.7; // Time in seconds to go from 0 to full throttle.
    public static final double kMaxSpeedMetersPerSecond = 5.22; // MtrPerSec 90% of 5.8 
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kWheelDiameterMeters = 0.2032;
    public static final double kDrivetrainGearRatio = 10.71;
    public static final double kEncoderRevToMeters = (kWheelDiameterMeters * Math.PI / kDrivetrainGearRatio);
    public static final double kEncoderRpmToMetersPerSecond = kEncoderRevToMeters / 60;

    // These are example values only for simple feedforward - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double kStaticGain = 0.1465;
    public static final double kVelocityGain = 2.0;
    public static final double kAccelerationGain = 1.5767;
    

    /** Example value only - as above, this must be tuned for your drive!
      Use kF to get the actual velocity to match the target velocity at steady state. With
      kF tuned, the top of the target velocity graph should match the actual velocity. However,
      there shold be a lot of "phase lag", where the accelerating/decelerating portions lag
      behind the target velocity. We will be raising the kP value to try and minimize this lag.

      Use kP to get the slope of the actual velocity to match the slope of the target velocity.
      Slowly increase the kP value to try and get the slopes to match the target. It is important
      to relize that raising kP will only get you so far in reducign the phase lag. There will be 
      a certain point where you keep raising it and nothing really imporves. You will just end up 
      increasing oscillations and only marginally reducing the phase lag. Do not try and tune to 
      perfection.

      Use kD to dampen any oscillations. Increasing kD too far will increase oscillations. You
      should really only need minot kD adjustments.

      Do not change kI. Adding an integral gain to the controller is an incorrect way to eliminate
      steady state error. A better approach would be to tune it with an integrator added to the plant,
      but this requires a model. Since we are doing output-based rather than model-based contorl, our
      only option is to add an integrator to the controller. If you feel the need to add kI, you should
      be increasing kF. 
    */
    public static final double kFFrontLeftVel = 0;
    public static final double kPFrontLeftVel = 0;
    public static final double kDFrontLeftVel = 0;
    public static final double kIFrontLeftVel = 0;

    public static final double kFRearLeftVel = 0;
    public static final double kPRearLeftVel = 0;
    public static final double kDRearLeftVel = 0;
    public static final double kIRearLeftVel = 0;

    public static final double kFFrontRightVel = 0;
    public static final double kPFrontRightVel = 0;
    public static final double kDFrontRightVel = 0;
    public static final double kIFrontRightVel = 0;

    public static final double kFRearRightVel = 0;
    public static final double kPRearRightVel = 0;
    public static final double kDRearRightVel = 0;
    public static final double kIRearRightVel = 0;

    public static final int kVelPidSlot = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kUpDPad = 0;
    public static final int kDownDPad = 180;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    public static final double kShooterAutonDelay = 3;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ShooterConstants {
    public static final double kLowShooterOutput = 0.25;
    public static final double kHighShooterOutput = 0.85;
    public static final int kShooterPort = 11;
    public static final boolean kShooterMotorInverted = true;
  }

  public static final class IntakeConstants {
    public static final int kIntakePort = 12;
    public static final double kIntakeOutput = 1;
    public static final boolean kIntakeMotorInverted = false;
  }

  public static final class ElevatorConstants {
    //1 = left motor, 2 = right motor
    public static final int kElevatorPort1 = 13;
    public static final int kElevatorPort2 = 14;
    public static final int kElevatorPort3 = 10;
    public static final double kElevatorOutput = 0.5;
    //inverted: left true = up, right true = down, left false = down, right false = up
    public static final boolean kElevatorMotor1Inverted = true;
    public static final boolean kElevatorMotor2Inverted = false;
    public static final boolean kElevatorMotor3Inverted = true;
    public static final double kElevatorMotor3Output = 0.25;
  }
}
