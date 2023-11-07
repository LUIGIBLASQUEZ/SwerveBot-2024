package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

// Legacy of 2024 team 691 controls team (we existed)

/*
 * Constants class holds all the variables, (such as motor ids or subsystem values)
 * Constants are static, but we can change them later
 * 
 */

public final class Constants {
    
    /*
     * Wrapper class for groups of constants
     * TODO: Divide up the DriveConstants class? There's far too much
     * (you have to change the instances they're called accordingly)
     * 
     */

    // Operator constants that are connected to controller imports (Robot container)
    public static final class OIConstants {
        public static final int kDriverControllerPortXbox = 0;
        public static final int kDRiverControllerPortStick = 1;
        public static final double kDriveDeadband = 0.05;
      }

    // Constants that handle values such as IDs and speeds
    public static class DriveConstants {
        /*
         * FOR MOTOR IDs:
         * L = Left
         * R = Right
         * F = Front
         * B = Back
         * Turn = Turning
         * Drive = Driving
         * 
         * turning motors are the secondary motor on a swerve module for turning (Mini NEO),
         * while driving motors are the primary motor for movement (Regular NEO)
         * 
         * also turning motors are ODD ids
         * and driving motors are EVEN ids
         */

        public static final int kLFTurnID = 1;
        public static final int kLFDriveID = 2;
        public static final int kRFTurnID = 3;
        public static final int kRFDriveID = 4;
        public static final int kLBTurnID = 5;
        public static final int kLBDriveID = 6;
        public static final int kRBTurnID = 7;
        public static final int kRBDriveID = 8;

        public static final boolean kGyroReversed = false;

        // Slew Rates and Speeds
        public static final double kMaxSpeedMetersPerSecond = 2.0; // Main translation speed value
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // Wheel measurements
        private static final double kWheelDiameter = 3.0; // inches
        private static final double kWheelCircumference = 9.43; // inches

        // Static motor values
        public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;
        public static final double kDrivingMotorReduction = (45.0 * 22) / (13 * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference) / kDrivingMotorReduction;

        // Static encoder values
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
        public static final boolean kTurningEncoderInverted = true;
        //private static final int kEncoderResolution = 4096;

        // Driving PIDs
        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        // Turning PIDs
        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        // Encoder PIDs
        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
        
        // Idle mode values
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        // AMP limit for motors (drivetrain)
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

        // Encoder factors
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameter * Math.PI)
        / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameter * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

        /* TODO: UPDATE THESE VALUES PLEASE */
        // Distance between left and right wheels (inches)
        public static final double kTrackWidth = 0;
        // Distance between front and back wheels (inches)
        public static final double kWheelBase = 0;

        // Kinematic values important for positioning, and odemetry
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase /  2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );
    }
}
