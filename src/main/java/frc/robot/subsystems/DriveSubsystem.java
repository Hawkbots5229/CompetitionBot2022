// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  // The front-left-side drive motor
  private final CANSparkMax m_frontLeft = 
      new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  // The rear-left-side drive motor
  private final CANSparkMax m_rearLeft = 
      new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  // The front-right-side drive motor
  private final CANSparkMax m_frontRight = 
      new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  // The rear-right-side drive motor
  private final CANSparkMax m_rearRight = 
      new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

  // The front-left-side drive encoder
  private final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  // The rear-left-side drive encoder
  private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
  // The front-right-side drive encoder
  private final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
  // The rear-right-side drive encoder
  private final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

  // Front left motor velocity PID controller
  private final SparkMaxPIDController m_frontLeftVelPIDController = m_frontLeft.getPIDController();
  // Rear left motor velocity PID controller
  private final SparkMaxPIDController m_rearLeftVelPIDController = m_rearLeft.getPIDController();
  // Front right motor velocity PID controller
  private final SparkMaxPIDController m_frontRightVelPIDController = m_frontRight.getPIDController();
  // Rear right motor velocity PID controller
  private final SparkMaxPIDController m_rearRightVelPIDController = m_rearRight.getPIDController();

  // Kauailabs navX-MXP motion processor
  private final AHRS m_gyro = 
      new AHRS(SPI.Port.kMXP);

  // Misc declarations
  private final SimpleMotorFeedforward m_feedForward;
  private final MecanumDrive m_drive;   

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    initMotors();
    initEncoders();
    initVelControl();

    m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
    m_feedForward = new SimpleMotorFeedforward(DriveConstants.kStaticGain, DriveConstants.kVelocityGain, DriveConstants.kAccelerationGain);
    m_drive.setExpiration(0.1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Wheel Velocity", m_frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Rear Left Wheel Velocity", m_rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Front Right Wheel Velocity", m_frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("Rear Right Wheel Velocity", m_rearRightEncoder.getVelocity());
    SmartDashboard.putNumber("Average Velocity", getRobotVelocity());
}

  private void initMotors() {
    
    m_frontLeft.restoreFactoryDefaults();
    m_rearLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults();
    m_rearRight.restoreFactoryDefaults();
    
    m_frontLeft.setIdleMode(DriveConstants.kIdleMode);
    m_rearLeft.setIdleMode(DriveConstants.kIdleMode);
    m_frontRight.setIdleMode(DriveConstants.kIdleMode);
    m_rearRight.setIdleMode(DriveConstants.kIdleMode);
    
    m_frontLeft.setInverted(DriveConstants.kFrontLeftMotorReversed);
    m_rearLeft.setInverted(DriveConstants.kRearLeftMotorReversed);
    m_frontRight.setInverted(DriveConstants.kFrontRightMotorReversed);
    m_rearRight.setInverted(DriveConstants.kRearRightMotorReversed);
    
    m_frontLeft.setOpenLoopRampRate(DriveConstants.kOpenLoopRampRate);
    m_rearLeft.setOpenLoopRampRate(DriveConstants.kOpenLoopRampRate);
    m_frontRight.setOpenLoopRampRate(DriveConstants.kOpenLoopRampRate);
    m_rearRight.setOpenLoopRampRate(DriveConstants.kOpenLoopRampRate); 
    
    m_frontLeft.setClosedLoopRampRate(DriveConstants.kClosedLoopRampRate);
    m_rearLeft.setClosedLoopRampRate(DriveConstants.kClosedLoopRampRate);
    m_frontRight.setClosedLoopRampRate(DriveConstants.kClosedLoopRampRate);
    m_rearRight.setClosedLoopRampRate(DriveConstants.kClosedLoopRampRate); 
    
    m_frontLeft.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
    m_rearLeft.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
    m_frontRight.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
    m_rearRight.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
  }

  private void initEncoders() {
    // Converts revolutions to meters
    m_frontLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderRevToMeters);
    m_rearLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderRevToMeters);
    m_frontRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderRevToMeters);
    m_rearRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderRevToMeters);

    // Converts RPM to meters per second
    m_frontLeftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderRpmToMetersPerSecond);
    m_rearLeftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderRpmToMetersPerSecond);
    m_frontRightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderRpmToMetersPerSecond);
    m_rearRightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderRpmToMetersPerSecond);  
    
    resetEncoders();

    // Note: SparkMax relateive encoders are inverted with motors. No action needed here.
  }

  private void initVelControl() {   
    
    m_frontLeftVelPIDController.setFF(DriveConstants.kFFrontLeftVel, DriveConstants.kVelPidSlot); 
    m_frontLeftVelPIDController.setP(DriveConstants.kPFrontLeftVel, DriveConstants.kVelPidSlot);
    m_frontLeftVelPIDController.setD(DriveConstants.kDFrontLeftVel, DriveConstants.kVelPidSlot);
    m_frontLeftVelPIDController.setI(DriveConstants.kIFrontLeftVel, DriveConstants.kVelPidSlot);

    m_rearLeftVelPIDController.setFF(DriveConstants.kFRearLeftVel, DriveConstants.kVelPidSlot);
    m_rearLeftVelPIDController.setP(DriveConstants.kPRearLeftVel, DriveConstants.kVelPidSlot);
    m_rearLeftVelPIDController.setD(DriveConstants.kDRearLeftVel, DriveConstants.kVelPidSlot);
    m_rearLeftVelPIDController.setI(DriveConstants.kIRearLeftVel, DriveConstants.kVelPidSlot);

    m_frontRightVelPIDController.setFF(DriveConstants.kFFrontRightVel, DriveConstants.kVelPidSlot);
    m_frontRightVelPIDController.setP(DriveConstants.kPFrontRightVel, DriveConstants.kVelPidSlot);
    m_frontRightVelPIDController.setD(DriveConstants.kDFrontRightVel, DriveConstants.kVelPidSlot);
    m_frontRightVelPIDController.setI(DriveConstants.kIFrontRightVel, DriveConstants.kVelPidSlot);

    m_rearRightVelPIDController.setFF(DriveConstants.kFRearRightVel, DriveConstants.kVelPidSlot);
    m_rearRightVelPIDController.setP(DriveConstants.kPRearRightVel, DriveConstants.kVelPidSlot);
    m_rearRightVelPIDController.setD(DriveConstants.kDRearRightVel, DriveConstants.kVelPidSlot);
    m_rearRightVelPIDController.setI(DriveConstants.kIRearRightVel, DriveConstants.kVelPidSlot);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param ySpeed Speed of the robot in the y direction (forward/backwards).
   * @param xSpeed Speed of the robot in the x direction (sideways). 
   * @param rot Angular rate of the robot. Clockwise is positive.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double ySpeed, double xSpeed, double rot, boolean fieldRelative) {
    m_drive.setSafetyEnabled(true);
    if (fieldRelative) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }

  }

  /**
   * Method to drive the robot using joystick info and PID control. Speeds range from [-1, 1].
   *
   * @param ySpeed Speed of the robot in the y direction (forward).
   * @param xSpeed Speed of the robot in the x direction (sideways).
   * @param rot Angular rate of the robot. Clockwise is positive.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drivePID(double ySpeed, double xSpeed, double rot, boolean fieldRelative) {
    m_drive.setSafetyEnabled(false);
    double xSpeedMeterPerSec = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedMeterPerSec = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotRadiansPerSec = rot * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
        DriveConstants.kDriveKinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeedMeterPerSec, xSpeedMeterPerSec, rotRadiansPerSec, m_gyro.getRotation2d())
                : new ChassisSpeeds(ySpeedMeterPerSec, xSpeedMeterPerSec, rotRadiansPerSec));

    mecanumDriveWheelSpeeds.desaturate(DriveConstants.kMaxSpeedMetersPerSecond);

    setSpeeds(mecanumDriveWheelSpeeds);
  }

   /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedForward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedForward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedForward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedForward.calculate(speeds.rearRightMetersPerSecond);

    m_frontLeftVelPIDController.setReference(
      speeds.frontLeftMetersPerSecond, 
      CANSparkMax.ControlType.kVelocity, 
      DriveConstants.kVelPidSlot, 
      frontLeftFeedforward);

    m_frontRightVelPIDController.setReference(
      speeds.frontRightMetersPerSecond, 
      CANSparkMax.ControlType.kVelocity, 
      DriveConstants.kVelPidSlot, 
      frontRightFeedforward);

    m_rearLeftVelPIDController.setReference(
      speeds.rearLeftMetersPerSecond, 
      CANSparkMax.ControlType.kVelocity, 
      DriveConstants.kVelPidSlot, 
      backLeftFeedforward);

    m_rearRightVelPIDController.setReference(
      speeds.rearRightMetersPerSecond, 
      CANSparkMax.ControlType.kVelocity, 
      DriveConstants.kVelPidSlot, 
      backRightFeedforward);
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }

  /** Stops all drive motors */
  public void stopMotors() {
    m_frontLeft.stopMotor();
    m_rearLeft.stopMotor();
    m_frontRight.stopMotor();
    m_rearRight.stopMotor();
  }

  /**
   * Gets drive velocity in meters per second by averaging each wheel velocity.
   * Only works in foreward and reverse.
   *
   * @return the robot's ground velocity
   */
  public double getRobotVelocity() {
    final double m_frontLeftVel = m_frontLeftEncoder.getVelocity();
    final double m_rearLeftVel = m_rearLeftEncoder.getVelocity();
    final double m_frontRightVel = m_frontRightEncoder.getVelocity();
    final double m_rearRightVel = m_rearRightEncoder.getVelocity();

    return (m_frontLeftVel + m_rearLeftVel + m_frontRightVel + m_rearRightVel) / 4;
  }

    /**
   * Gets drive velocity in meters per second by averaging each wheel velocity.
   * Only works in foreward and reverse.
   *
   * @return the robot's ground velocity
   */
  public MecanumDriveWheelSpeeds getWheelVelocities() {
    final double m_frontLeftVel = m_frontLeftEncoder.getVelocity();
    final double m_rearLeftVel = m_rearLeftEncoder.getVelocity();
    final double m_frontRightVel = m_frontRightEncoder.getVelocity();
    final double m_rearRightVel = m_rearRightEncoder.getVelocity();

    return new MecanumDriveWheelSpeeds(m_frontLeftVel, m_frontRightVel, m_rearLeftVel, m_rearRightVel);
  }

  /**
   * Gets robot position in meters by averaging each wheel position.
   * Does not indicate direction.
   *
   * @return the robot's position
   */
  public double getRobotPosition() {
    final double m_frontLeftPos = Math.abs(m_frontLeftEncoder.getPosition());
    final double m_rearLeftPos = Math.abs(m_rearLeftEncoder.getPosition());
    final double m_frontRightPos = Math.abs(m_frontRightEncoder.getPosition());
    final double m_rearRightPos = Math.abs(m_rearRightEncoder.getPosition());

    return (m_frontLeftPos + m_rearLeftPos + m_frontRightPos + m_rearRightPos) / 4;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_rearLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_rearRightEncoder.getVelocity());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {

    if (DriveConstants.kGyroInverted) {
      return -m_gyro.getRate();
    } else {
      return m_gyro.getRate();
    }
  }

    /**
   * Returns the turn rate of the robot.
   *
   * @return The yaw of the robot, in degrees
   */
  public double getAngle() {

    if (DriveConstants.kGyroInverted) {
      return -m_gyro.getAngle();
    } else {
      return m_gyro.getAngle();
    }
  }
}
