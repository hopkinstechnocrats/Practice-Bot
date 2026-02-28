package frc.robot.swerve;


import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase{
    
    AHRS navx;
    

    Gyro(){
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    }

    public Rotation2d getRotation(){
        return navx.getRotation2d();
    }

}
