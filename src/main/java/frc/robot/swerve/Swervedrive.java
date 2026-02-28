package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import frc.robot.Constants;

public class Swervedrive extends SubsystemBase{
    
    SwerveDriveKinematics m_swerveKinematics;
    Translation2d m_frontLeftPosition;
    Translation2d m_frontRightPosition;
    Translation2d m_backLeftPosition;
    Translation2d m_backRightPosition;

    SwerveDriveOdometry swerveOdometry;
    Pose2d m_pose;
    
    NetworkTableInstance inst;
    NetworkTable table;

    StructArrayPublisher<SwerveModuleState> desiredStatePublisher;
    StructArrayPublisher<SwerveModuleState> actualStatePublisher;

    DoubleEntry flAnalog;
    DoubleEntry frAnalog;
    DoubleEntry blAnalog;
    DoubleEntry brAnalog;

    SwerveModule fL;
    SwerveModule fR;
    SwerveModule bL;
    SwerveModule bR;

    Gyro gyro;

    SwerveModuleState[] desiredModuleStates;
    SwerveModuleState[] actualModuleState = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    ChassisSpeeds m_speeds;

    public Swervedrive(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Swerve");

        desiredStatePublisher = table.getStructArrayTopic("Desired Module States", SwerveModuleState.struct).publish();
        actualStatePublisher = table.getStructArrayTopic("Actual Module States", SwerveModuleState.struct).publish();

        m_frontLeftPosition = new Translation2d(Constants.SwerveConstants.frontLeftX, Constants.SwerveConstants.frontLeftY);
        m_frontRightPosition = new Translation2d(Constants.SwerveConstants.frontRightX, Constants.SwerveConstants.frontRightY);
        m_backLeftPosition = new Translation2d(Constants.SwerveConstants.backLeftX, Constants.SwerveConstants.backLeftY);
        m_backRightPosition = new Translation2d(Constants.SwerveConstants.backRightX, Constants.SwerveConstants.backRightY);

        fL = new SwerveModule(Constants.SwerveConstants.k_frontLeftDriveCANID, Constants.SwerveConstants.k_frontLeftTurnCANID, 
                Constants.SwerveConstants.k_flAbsEncoderPort, Constants.SwerveConstants.k_flAbsEncoderOffset);
        fR = new SwerveModule(Constants.SwerveConstants.k_frontRightDriveCANID, Constants.SwerveConstants.k_frontRightTurnCANID, 
                Constants.SwerveConstants.k_frAbsEncoderPort, Constants.SwerveConstants.k_frAbsEncoderOffset);
        bL = new SwerveModule(Constants.SwerveConstants.k_backLeftDriveCANID, Constants.SwerveConstants.k_backLeftTurnCANID,
                Constants.SwerveConstants.k_blAbsEncoderPort, Constants.SwerveConstants.k_blAbsEncoderOffset);
        bR = new SwerveModule(Constants.SwerveConstants.k_backRightDriveCANID, Constants.SwerveConstants.k_backRightTurnCANID,
                Constants.SwerveConstants.k_brAbsEncoderPort, Constants.SwerveConstants.k_brAbsEncoderOffset);

        m_swerveKinematics = new SwerveDriveKinematics(m_frontLeftPosition, m_frontRightPosition, m_backLeftPosition, m_backRightPosition);

        gyro = new Gyro(Constants.GyroConstants.k_gyroID);
        swerveOdometry = new SwerveDriveOdometry(m_swerveKinematics, gyro.getRotation(), new SwerveModulePosition[]{
            fL.getModulePosition(), fR.getModulePosition(), bL.getModulePosition(), bR.getModulePosition()
        }, Constants.SwerveConstants.k_startPose);

        flAnalog = table.getDoubleTopic("FL Absolute Encoder").getEntry(0);
        frAnalog = table.getDoubleTopic("FR Absolute Encoder").getEntry(0);
        blAnalog = table.getDoubleTopic("BL Absolute Encoder").getEntry(0);
        brAnalog = table.getDoubleTopic("BR Absolute Encoder").getEntry(0);
    }


    @Override
    public void periodic(){
        m_pose = swerveOdometry.update(gyro.getRotation(), new SwerveModulePosition[]{
             fL.getModulePosition(), fR.getModulePosition(), bL.getModulePosition(), bR.getModulePosition()
        });

        desiredStatePublisher.set(desiredModuleStates);

        updateActualStates();

        actualStatePublisher.set(actualModuleState);

        flAnalog.set(fL.getAbsEncoderPositionRot());
        frAnalog.set(fR.getAbsEncoderPositionRot());
        blAnalog.set(bL.getAbsEncoderPositionRot());
        brAnalog.set(bR.getAbsEncoderPositionRot());
    }

    public void Drive(ChassisSpeeds desiredState){
        desiredModuleStates = m_swerveKinematics.toSwerveModuleStates(desiredState);
        fL.Drive(desiredModuleStates[0]);
        fR.Drive(desiredModuleStates[1]);
        bL.Drive(desiredModuleStates[2]);
        bR.Drive(desiredModuleStates[3]);
    }

    
    private void updateActualStates(){
        actualModuleState[0] = new SwerveModuleState(fL.getDriveVelocityMeterPerSec(), fL.getAngleRotation2d());
        actualModuleState[1] = new SwerveModuleState(fR.getDriveVelocityMeterPerSec(), fR.getAngleRotation2d());
        actualModuleState[2] = new SwerveModuleState(bL.getDriveVelocityMeterPerSec(), bL.getAngleRotation2d());
        actualModuleState[3] = new SwerveModuleState(bR.getDriveVelocityMeterPerSec(), bR.getAngleRotation2d());

    }
    

    public Rotation2d getRotation(){
        return gyro.getRotation();
    }
}
