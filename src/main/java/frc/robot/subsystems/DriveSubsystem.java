// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

  private final MecanumDrive m_drive =
      new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  // The front-left-side drive encoder
  private final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();

  // The rear-left-side drive encoder
  private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();

  // The front-right--side drive encoder
  private final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();

  // The rear-right-side drive encoder
  private final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

  // The gyro sensor
  private final AHRS m_gyro = 
      new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final MecanumDriveOdometry m_odometry = 
      new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  // Front left motor velocity PID controller
  private final PIDController m_frontLeftVelPIDController = 
      new PIDController(DriveConstants.kPFrontLeftVel, DriveConstants.kDFrontLeftVel, DriveConstants.kIFrontLeftVel);

  // Rear left motor velocity PID controller
  private final PIDController m_rearLeftVelPIDController = 
      new PIDController(DriveConstants.kPRearLeftVel, DriveConstants.kDRearLeftVel, DriveConstants.kIRearLeftVel);

  // Front right motor velocity PID controller
  private final PIDController m_frontRightVelPIDController = 
      new PIDController(DriveConstants.kPFrontRightVel, DriveConstants.kDFrontRightVel, DriveConstants.kIFrontRightVel);

  // Rear right motor velocity PID controller
  private final PIDController m_rearRightVelPIDController = 
      new PIDController(DriveConstants.kPRearRightVel, DriveConstants.kDRearRightVel, DriveConstants.kIRearRightVel);

  private final SimpleMotorFeedforward m_feedForward =
        new SimpleMotorFeedforward(DriveConstants.kStaticGain, DriveConstants.kVelocityGain, DriveConstants.kAccelerationGain);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Converts revolutions to meters
    m_frontLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRev);
    m_rearLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRev);
    m_frontRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRev);
    m_rearRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRev);

    // Converst RPM to meters per second
    m_frontLeftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRev/60);
    m_rearLeftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRev/60);
    m_frontRightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRev/60);
    m_rearRightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerRev/60);  

    // Set the phase of the MotorFeedbackSensor so that it is set to be in phase with the motor itself.
    m_frontLeftEncoder.setInverted(DriveConstants.kFrontLeftEncoderReversed);
    m_rearLeftEncoder.setInverted(DriveConstants.kRearLeftEncoderReversed);
    m_frontRightEncoder.setInverted(DriveConstants.kFrontRightEncoderReversed);
    m_rearRightEncoder.setInverted(DriveConstants.kRearRightEncoderReversed);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontLeft.setInverted(DriveConstants.kFrontLeftMotorReversed);
    m_rearLeft.setInverted(DriveConstants.kRearLeftMotorReversed);
    m_frontRight.setInverted(DriveConstants.kFrontRightMotorReversed);
    m_rearRight.setInverted(DriveConstants.kRearRightMotorReversed);
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

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }
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

    final double frontLeftOutput =
        m_frontLeftVelPIDController.calculate(
            m_frontLeftEncoder.getVelocity(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        m_frontRightVelPIDController.calculate(
            m_frontRightEncoder.getVelocity(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        m_rearLeftVelPIDController.calculate(
            m_rearLeftEncoder.getVelocity(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        m_rearRightVelPIDController.calculate(
            m_rearRightEncoder.getVelocity(), speeds.rearRightMetersPerSecond);

    m_frontLeft.setVoltage(frontLeftOutput + frontLeftFeedforward);
    m_frontRight.setVoltage(frontRightOutput + frontRightFeedforward);
    m_rearLeft.setVoltage(backLeftOutput + backLeftFeedforward);
    m_rearRight.setVoltage(backRightOutput + backRightFeedforward);
  }

  /**
   * Method to drive the robot using joystick info and PID control.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drivePID(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mecanumDriveWheelSpeeds =
        DriveConstants.kDriveKinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    mecanumDriveWheelSpeeds.desaturate(DriveConstants.kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
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
    m_frontRightEncoder.setPosition(0);;
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
   * Gets the front left motor velocity PID Controller.
   *
   * @return the front left motor velocity PID Controller
   */
  public PIDController getFrontLeftVelPIDController() {
    return m_frontLeftVelPIDController;
  }

  /**
   * Gets the rear left motor velocity PID Controller.
   *
   * @return the rear left motor velocity PID Controller
   */
  public PIDController getRearLeftVelPIDController() {
    return m_rearLeftVelPIDController;
  }

  /**
   * Gets the front right motor velocity PID Controller.
   *
   * @return the front right motor velocity PID Controller
   */
  public PIDController getFrontRightVelPIDController() {
    return m_frontRightVelPIDController;
  }

  /**
   * Gets the rear right motor velocity PID Controller.
   *
   * @return the rear right motor velocity PID Controller
   */
  public PIDController getRearRightVelPIDController() {
    return m_rearRightVelPIDController;
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
}
