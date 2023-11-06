package frc.robot.subsystems;

/* IMPORTS */

// Odemetry
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// Encoders
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
// Motor Controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
// Constants
import frc.robot.Constants.DriveConstants;

/*
 * Class handling REV Robotics SwerveModules
 * Each swerve module contains one driving motor, and one turning motor that work together
 * 
 */

public class MAXSwerveModules {

    // Motor types on each swerve module
    private final CANSparkMax m_drivingSparkMax; // Regular NEO
    private final CANSparkMax m_turningSparkMax; // Mini NEO
    // Encoders for the swerve module motors
    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;
    // PIDs for the motor controllers
    private final SparkMaxPIDController m_drivingPIDController;
    private final SparkMaxPIDController m_turningPIDController;

    // Offset Value
    private double m_chassisAngularOffset = 0;
    // Handles the position and state of a swerve module
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    // Swerve module constructor
    public MAXSwerveModules(int drivingCANId, int turningCANId, double chassisAngularOffset) {
  
    // Objects for SparkMax Motor controllers, specifically driving, and turning motors
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset to get SparkMax controllers to a default state
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Conversion factors for driving factors from rpm to m/s
    m_drivingEncoder.setPositionConversionFactor(DriveConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(DriveConstants.kDrivingEncoderVelocityFactor);

    // Conversion factors for driving factors from EMPTY to radians
    m_turningEncoder.setPositionConversionFactor(DriveConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(DriveConstants.kTurningEncoderVelocityFactor);

    // Sets the turning motor to register as inverted (IMPORTANT)
    m_turningEncoder.setInverted(DriveConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(DriveConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(DriveConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor
    m_drivingPIDController.setP(DriveConstants.kDrivingP);
    m_drivingPIDController.setI(DriveConstants.kDrivingI);
    m_drivingPIDController.setD(DriveConstants.kDrivingD);
    m_drivingPIDController.setFF(DriveConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(DriveConstants.kDrivingMinOutput,
    DriveConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor
    m_turningPIDController.setP(DriveConstants.kTurningP);
    m_turningPIDController.setI(DriveConstants.kTurningI);
    m_turningPIDController.setD(DriveConstants.kTurningD);
    m_turningPIDController.setFF(DriveConstants.kTurningFF);
    m_turningPIDController.setOutputRange(DriveConstants.kTurningMinOutput,
    DriveConstants.kTurningMaxOutput);

    // Sets turning and driving motor to idle mode
    m_drivingSparkMax.setIdleMode(DriveConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(DriveConstants.kTurningMotorIdleMode);

    // Current limit for motors (amps)
    m_drivingSparkMax.setSmartCurrentLimit(DriveConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(DriveConstants.kTurningMotorCurrentLimit);

    // Configs for motor controllers are preserved, useful for if a motor controller burns out
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    // Declarative odemetry misc.
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }
  
  // Method that gets encoder states relative to the chassis
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
           new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // Method that gets encoder positions relative to the chassis
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_drivingEncoder.getPosition(),
           new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // Method that sets the desired states for a module, as well as applies chassis angular offset to the desired state
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Reference state is optimized to stop spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Sets driving and turning motors to respective setpoints
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  // Sets all swerve module encoders to 0
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
