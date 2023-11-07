package frc.robot.subsystems;

/* IMPORTS */

// Odemetry
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// Utilities
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.util.WPIUtilJNI;
// Gyro
import edu.wpi.first.wpilibj.ADIS16470_IMU;
// Constants
import frc.robot.Constants.DriveConstants;
// Subsystem Base
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Drive train of the robot, currently a swerve drive framework with REV modules
 * Main method of robot field movement
 * 
 */

public class DriveTrain extends SubsystemBase{

  // REV Robotics Swerve Modules objects
     public final MAXSwerveModules m_frontLeft = new MAXSwerveModules(
         DriveConstants.kLFDriveID,
         DriveConstants.kLFTurnID,
         DriveConstants.kFrontLeftChassisAngularOffset);
    
    private final MAXSwerveModules m_frontRight = new MAXSwerveModules(
        DriveConstants.kRFDriveID,
        DriveConstants.kRFTurnID,
        DriveConstants.kFrontRightChassisAngularOffset);
  
    private final MAXSwerveModules m_rearLeft = new MAXSwerveModules(
        DriveConstants.kLBDriveID,
        DriveConstants.kLBTurnID,
        DriveConstants.kBackLeftChassisAngularOffset);
  
    private final MAXSwerveModules m_rearRight = new MAXSwerveModules(
        DriveConstants.kRBDriveID,
        DriveConstants.kRBTurnID,
        DriveConstants.kBackRightChassisAngularOffset);

    // Gyro object
    // TODO: MANUFACTURING - Get gyro attached to ROBORio
    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    /* Swerve Drive Odometry */
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry
    (DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getAngle()),new SwerveModulePosition[] 
      {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      }
    );

    // Resets odemetry to desired position
    public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(
          Rotation2d.fromDegrees(m_gyro.getAngle()),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          } ,pose
      );
    }

    // Method to drive robot using Joystick info
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
      // X is driving motor speed while Y is turning motor speed
      double xSpeedCommanded;
      double ySpeedCommanded;

      // Grants smoother handling
      if (rateLimit) {
          // Convert XY to polar for rate limiting
          double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
          double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
  
          // Calculate the direction slew rate based on an estimate of the lateral acceleration
          double directionSlewRate;
          if (m_currentTranslationMag != 0.0) {
              directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
          } 
          else {
              directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
          }
        
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
        if (angleDif < 0.45*Math.PI) {
          m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
        else if (angleDif > 0.85*Math.PI) {
          if (m_currentTranslationMag > 1e-4) { 
            // keep currentTranslationDir unchanged
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
          }
          else {
            m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
          }
        }
        else {
          m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        m_prevTime = currentTime;
        
        xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
        ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
        m_currentRotation = m_rotLimiter.calculate(rot);
  
  
      } else {
        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;
        m_currentRotation = rot;
      }
  
      // Unit conversion for commanded speeds for the drivetrain
      double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
      double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
  
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle()))
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // Sets Swerve Module states
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
    }

    // Sets all drive encoders to return a position of 0
    public void resetEncoders() {
      m_frontLeft.resetEncoders();
      m_rearLeft.resetEncoders();
      m_frontRight.resetEncoders();
      m_rearRight.resetEncoders();
    }

    // Zeros heading of the robot
    public void zeroHeading() {
      m_gyro.reset();
    }

    // Returns heading of the robot
    public double getHeading() {
      return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }

    // Returns turn rate of the robot
    public double getTurnRate() {
      return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /*
     * Extra DriveTrain commands
     * Bound to JoystickButton
     * 
     */

    // Turns all wheels into an X as well as speed to 0 to stop movement
    public void stop() {
      m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    // !!BELOW REQUIRE TESTING, ESPECIALLY THE ANGLE "null" VALUES!!

    // Slows down translation speed while button is held down
    public void slower() {
      m_frontLeft.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond*0.5, null));
      m_frontRight.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond*0.5, null));
      m_rearLeft.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond*0.5, null));
      m_rearRight.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond*0.5, null)); 
    }

    // Speeds up translation speed while button is held down
    public void faster() {
      m_frontLeft.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond*1.5, null));
      m_frontRight.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond*1.5, null));
      m_rearLeft.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond*1.5, null));
      m_rearRight.setDesiredState(new SwerveModuleState(DriveConstants.kMaxSpeedMetersPerSecond*1.5, null)); 
    }
}
