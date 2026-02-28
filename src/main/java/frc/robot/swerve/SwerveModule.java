package frc.robot.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    TalonFX m_driveMotor;
    TalonFX m_turnMotor;

    TalonFXConfiguration m_driveConfig;
    TalonFXConfiguration m_turnConfig;

    AnalogEncoder m_absoluteEncoder;

    final PositionVoltage m_turnRequest = new PositionVoltage(0).withSlot(0);
    final VelocityVoltage m_driveRequest = new VelocityVoltage(0).withSlot(0);

    SwerveModuleState m_moduleState;

    SwerveModule(int driveID, int turnID, int absEncoderPort, double absEcoderOffset){
        m_driveMotor = new TalonFX(driveID, new CANBus("GertrudeGreyser"));
        m_turnMotor = new TalonFX(turnID, new CANBus("GertrudeGreyser"));

        m_absoluteEncoder = new AnalogEncoder(absEncoderPort);

        m_driveConfig = new TalonFXConfiguration();
        m_turnConfig = new TalonFXConfiguration();


        m_driveConfig.Slot0.kP = Constants.SwerveConstants.k_driveKP;
        m_driveConfig.Slot0.kI = Constants.SwerveConstants.k_driveKI;
        m_driveConfig.Slot0.kD = Constants.SwerveConstants.k_driveKD;

        m_turnConfig.Slot0.kP = Constants.SwerveConstants.k_turnKP;
        m_turnConfig.Slot0.kI = Constants.SwerveConstants.k_turnKI;
        m_turnConfig.Slot0.kD = Constants.SwerveConstants.k_turnKD;


        //TODO I have no clue something with inversion
        m_turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_turnConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.k_turnGearRatio;
        m_turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        m_driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_driveMotor.getConfigurator().apply(m_driveConfig);
        m_turnMotor.getConfigurator().apply(m_turnConfig);

        m_turnMotor.getConfigurator().setPosition((m_absoluteEncoder.get()-absEcoderOffset)*Constants.SwerveConstants.k_turnGearRatio);
    }

    public void Drive(SwerveModuleState moduleState){
        m_moduleState = moduleState;
        m_moduleState.optimize(this.getAngleRotation2d());
        m_moduleState.speedMetersPerSecond *= m_moduleState.angle.minus(this.getAngleRotation2d()).getCos();
        m_driveMotor.setControl(m_driveRequest.withVelocity(m_moduleState.speedMetersPerSecond * Constants.SwerveConstants.k_driveGearRatio));
        m_turnMotor.setControl(m_turnRequest.withPosition(m_moduleState.angle.getRotations()));
    }

    public double getAnglePositionRot(){
        return m_turnMotor.getPosition().getValueAsDouble()/Constants.SwerveConstants.k_turnGearRatio;
    }

    public double getDrivePositionRot(){
        return m_driveMotor.getPosition().getValueAsDouble()/Constants.SwerveConstants.k_driveGearRatio;
    }

    public double getDriveDistanceMeters(){
        return this.getDrivePositionRot()*Constants.SwerveConstants.k_wheelCircumferenceMeters;
    }

    public Rotation2d getAngleRotation2d(){
        return new Rotation2d((m_turnMotor.getPosition().getValueAsDouble() / Constants.SwerveConstants.k_turnGearRatio)* Math.PI * 2); 
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(this.getDriveDistanceMeters(), this.getAngleRotation2d());
    }

    public double getAbsEncoderPositionRot(){
        return m_absoluteEncoder.get();
    }

    public double getDriveVelocityMeterPerSec(){
        return (m_driveMotor.getVelocity().getValueAsDouble()/Constants.SwerveConstants.k_driveGearRatio) * Constants.SwerveConstants.k_wheelCircumferenceMeters;
    }
}
