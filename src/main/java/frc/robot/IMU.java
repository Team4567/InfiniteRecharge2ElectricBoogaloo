package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

// The current PigeonIMU Class did not have the ability to send its data to the SmartDashboard (Will see)
// I made this to do it for me :)
public class IMU extends PigeonIMU implements Sendable  {
    // Connects this class to its parent PigeonIMU class
    // See video if confused
    // 
    boolean isNegative = false;
    public IMU(int port) {
        super(port);
    }
    public IMU(TalonSRX talon){
        super(talon);
    }

    // Gives the rotation value relative to the start of game
    // Will go past 360 degrees, ex 2 full rotations will be 720 degrees
    public double yawCont(){
        double[] ypr = {0,0,0};
        getYawPitchRoll(ypr);
        return ypr[0];
    }

    // Returns the rotation of the gyro from 0 - 359
    // The yawCont function will accumulate degrees and go over 1 rotation
    // Ex 2 full rotations will read 720 instead of 0
    // This accounts for that by using the Mod Operator (%)
    // Essentially, a % b returns the remainder of a / b, if you remember remainders from 5th grade.
    public double yaw(){
        return yawCont() % 360;
    }

    public double yawWPI(){
        double y = yaw();
        if( y > 180 ){
            isNegative = true;
            return -( 360 - y );
            
        }else if( y < 180 ){
            isNegative = false;
            return y;
        }
        return 180 * ( isNegative ? -1 : 1 );
    }

    // Converts the gyro value to a Rotation2d class, used for trajectories
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees( yawCont() );
    }

    // Reset the gyro value to 0
    public void reset(){
        setYaw(0);
    }

    // Sends the degrees per second of the gyro
    public double getYawRate(){
        double[] ypr = {0,0,0};
        getRawGyro(ypr);
        return ypr[2];
    }

    /** Code to send gyro to Shuffleboard Widget 
     * For example, see 
     * @link https://prnt.sc/wlehha
     * */ 
    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::yaw, null);
    }
   
}