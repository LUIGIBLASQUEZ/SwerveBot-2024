// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.DriveConstants;

public class SwerveModules {
  /*
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared
  

  private final MotorController m_driveMotor;
  private final MotorController m_turningMotor;

  private final Encoder m_driveEncoder;
  private final Encoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);
  */          
  public class MAXSwerveModules {
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkMaxPIDController m_drivingPIDController;
    private final SparkMaxPIDController m_turningPIDController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    /* 
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderChannelA,
      int turningEncoderChannelB) {
    m_driveMotor = new PWMSparkMax(driveMotorChannel);
    m_turningMotor = new PWMSparkMax(turningMotorChannel);

    m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    */

    //FRONT
    //private final CANSparkMax m_frontLeftTurn = new CANSparkMax(DriveConstants.kLFTurnID, MotorType.kBrushless);
    //private final CANSparkMax m_frontLeftDrive = new CANSparkMax(DriveConstants.kLFDriveID, MotorType.kBrushless);
    //private final CANSparkMax m_frontRightTurn = new CANSparkMax(DriveConstants.kRFTurnID, MotorType.kBrushless);
    //private final CANSparkMax m_frontRightDrive = new CANSparkMax(DriveConstants.kRFDriveID, MotorType.kBrushless);
    //BACK
    //private final CANSparkMax m_backLeftTurn = new CANSparkMax(DriveConstants.kLBTurnID, MotorType.kBrushless);
    //private final CANSparkMax m_backLeftDrive = new CANSparkMax(DriveConstants.kLBDriveID, MotorType.kBrushless);
    //private final CANSparkMax m_backRightTurn = new CANSparkMax(DriveConstants.kRBTurnID, MotorType.kBrushless);
    //private final CANSparkMax m_backRightDrive = new CANSparkMax(DriveConstants.kRBDriveID, MotorType.kBrushless);
      /**
   * Constructs a SwerveModules and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */

    //PULLED DIRECTLY FROM REV EXAMPLE SWERVE
    public MAXSwerveModules(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(DriveConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(DriveConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(DriveConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(DriveConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(DriveConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(DriveConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(DriveConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(DriveConstants.kDrivingP);
    m_drivingPIDController.setI(DriveConstants.kDrivingI);
    m_drivingPIDController.setD(DriveConstants.kDrivingD);
    m_drivingPIDController.setFF(DriveConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(DriveConstants.kDrivingMinOutput,
        DriveConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(DriveConstants.kTurningP);
    m_turningPIDController.setI(DriveConstants.kTurningI);
    m_turningPIDController.setD(DriveConstants.kTurningD);
    m_turningPIDController.setFF(DriveConstants.kTurningFF);
    m_turningPIDController.setOutputRange(DriveConstants.kTurningMinOutput,
        DriveConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(DriveConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(DriveConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(DriveConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(DriveConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }
  
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
}
