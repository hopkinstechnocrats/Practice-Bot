package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants{

    public static final class SwerveConstants{
        public static final int k_frontLeftDriveCANID = 3;
        public static final int k_frontRightDriveCANID = 5;
        public static final int k_backLeftDriveCANID = 9;
        public static final int k_backRightDriveCANID = 7;

        public static final int k_frontLeftTurnCANID = 4;
        public static final int k_frontRightTurnCANID = 6;
        public static final int k_backLeftTurnCANID = 10;
        public static final int k_backRightTurnCANID = 8;

        public static final int k_flAbsEncoderPort = 0;
        public static final int k_frAbsEncoderPort = 1;
        public static final int k_blAbsEncoderPort = 2;
        public static final int k_brAbsEncoderPort = 3;

        public static final double k_driveKP = 0.3;
        public static final double k_driveKI = 0.0;
        public static final double k_driveKD = 0.0;

        public static final double k_turnKP = 0.5;
        public static final double k_turnKI = 0.0;
        public static final double k_turnKD = 0.0;

        public static final boolean k_turnInverted = true;
        public static final boolean k_driveInverted = false;

        public static final double frontLeftX = 0.3;
        public static final double frontLeftY = 0.3;
        public static final double frontRightX = 0.3;
        public static final double frontRightY = -0.3;
        public static final double backLeftX = -0.3;
        public static final double backLeftY = 0.3;
        public static final double backRightX = -0.3;
        public static final double backRightY = -0.3;

        public static final double k_flAbsEncoderOffset = 0.942;
        public static final double k_frAbsEncoderOffset = 0.76;
        public static final double k_blAbsEncoderOffset = 0.603;
        public static final double k_brAbsEncoderOffset = 0.120;

        public static final double k_maxLinearSpeedMeterPerSecond = 3;
        public static final double k_maxAngularSpeedRadPerSec = 4.0 * Math.PI;

        public static final double k_driveGearRatio = 6.75;
        public static final double k_turnGearRatio = 12.8;
        public static final double k_wheelCircumferenceMeters = 0.1016 * Math.PI;

        public static final Pose2d k_startPose = new Pose2d(0, 0, new Rotation2d(0));
    }
 
    public static final class ControlConstants{
        public static final double k_driveControllerDeadband = 0.1;
        public static final double k_operatorControllerDeadband = 0.1;
        public static final int k_driverPort = 0;
    }
    
    public static final class GyroConstants{
        public static final int k_gyroID = 15;
    }


}
