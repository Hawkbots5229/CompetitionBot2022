// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax m_frontLeft = 
      new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearLeft = 
      new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_frontRight = 
      new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearRight = 
      new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

  // The front-left-side drive encoder
  private final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();

  // The rear-left-side drive encoder
  private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();

  // The front-right--side drive encoder
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

  // The gyro sensor
  private final AHRS m_gyro = 
      new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final MecanumDriveOdometry m_odometry = 
      new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  private final SimpleMotorFeedforward m_feedForward =
        new SimpleMotorFeedforward(DriveConstants.kStaticGain, DriveConstants.kVelocityGain, DriveConstants.kAccelerationGain);

  private final MecanumDrive m_drive;   

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    initMotors();
    //initEncoders();
    //initVelPIDControllers();
    m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new MecanumDriveWheelSpeeds(
            m_frontLeftEncoder.getVelocity(),
            m_rearLeftEncoder.getVelocity(),
            m_frontRightEncoder.getVelocity(),
            m_rearRightEncoder.getVelocity()));
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

  private void initVelPIDControllers() {   
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
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double ySpeed, double xSpeed, double rot, boolean fieldRelative) {

    if (fieldRelative) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }

  }

  /**
   * Method to drive the robot using joystick info and PID control. Speeds range from [-1, 1].
   *
   * @param ySpeed Speed of the robot in the x direction (forward).
   * @param xSpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drivePID(double ySpeed, double xSpeed, double rot, boolean fieldRelative) {
    double xSpeedMeterPerSec = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedMeterPerSec = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotRadiansPerSec = rot * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
        DriveConstants.kDriveKinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMeterPerSec, ySpeedMeterPerSec, rotRadiansPerSec, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeedMeterPerSec, ySpeedMeterPerSec, rotRadiansPerSec));

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

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public RelativeEncoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public RelativeEncoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public RelativeEncoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public RelativeEncoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }

    /**
   * Gets simple feedforward object for drive motors.
   *
   * @return the simple feedforward object for drive motors.
   */
  public SimpleMotorFeedforward getFeedForward() {
    return m_feedForward;
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
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
}
