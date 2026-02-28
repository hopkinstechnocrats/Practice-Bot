package frc.robot.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase{
    
    Pigeon2 pigeon;

    Gyro(int gyroID){
        pigeon = new Pigeon2(gyroID);
    }

    public Rotation2d getRotation(){
        return pigeon.getRotation2d();
    }

    public double getAccelX(){
        return pigeon.getAccelerationX().getValueAsDouble();
    }

    public double getAccelY(){
        return pigeon.getAccelerationY().getValueAsDouble();
    }

    public double getAccelZ(){
        return pigeon.getAccelerationZ().getValueAsDouble();
    }

}
