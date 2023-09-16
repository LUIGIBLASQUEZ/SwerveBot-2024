package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveModules.MAXSwerveModules;
//Constant imports??
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase{

    private final MAXSwerveModules m_frontLeft = new MAXSwerveModules(
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

    /* USE IF USING NAVX
     *
    public Rotation2d getHeadingRotation2d() {
      return Rotation2d.fromDegrees(getHeadingDegrees());
      }
      public double getHeadingDegrees() {
        try {
            return Math.IEEEremainder(-mNavX.getAngle(), 360);
        } catch (Exception e) {
            System.out.println("Cannot Get NavX Heading");
            return 0;
        }
      }
    */

    //gyro
    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

        /*
         * Swerve Drive Odometry
         * 
         */
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  });
  
}
