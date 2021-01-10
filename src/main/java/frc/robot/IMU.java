package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

// The current PigeonIMU Class did not have the ability to send its data to the SmartDashboard (Will see)
// I made this to do it for me :)
public class IMU extends PigeonIMU implements Sendable  {
    public IMU(int port) {
        super(port);
    }
    public IMU(TalonSRX talon){
        super(talon);
    }
    public double yawCont(){
        double[] ypr = {0,0,0};
        getYawPitchRoll(ypr);
        return ypr[0];
    }
    public double yaw(){
        return yawCont() % 360;
    }
    public Rotation2d getRotation2d(){
        return new Rotation2d( Math.toRadians( yaw() ) );
    }
    public void reset(){
        setYaw(0);
    }
    public double getYawRate(){
        double[] ypr = {0,0,0};
        getRawGyro(ypr);
        return ypr[0];
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::yaw, null);
    }
   
}