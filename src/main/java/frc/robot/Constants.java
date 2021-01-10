/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
// Its RobotMap, but I named it Constants
public final class Constants {
    /**
     * CAN/PWM Values for everything
     */
    public static final int kCANLMaster = 3;
    public static final int kCANLSlave = 0;
    public static final int kCANRMaster = 2;
    public static final int kCANRSlave = 1;

	public static final int kCANIntake = 0;
	public static final int kCANFlip = 15;

	public static final int kCANClimb = 1;

	public static final int kCANVictorControl = -1; 
	public static final int kPCMControlIn = 4;
	public static final int kPCMControlOut = 5;

	public static final int kCANIMU = 5;
	
	public static final int kCANPCMA = 10;
	
	public static final int kPCMLGearboxIn = 0;
	public static final int kPCMLGearboxOut = 1;
	public static final int kPCMRGearboxIn = 2;
	public static final int kPCMRGearboxOut = 3;
	
	/**
	 * This is a property of the Pigeon IMU, and should not be changed.
	 */
	public final static int kPigeonUnitsPerRotation = 8192;





	public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
	public static final double kPDriveVel = 8.5;
	public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
		new DifferentialDriveKinematics( kTrackwidthMeters );
		
	public static final double kMaxSpeedMetersPerSecond = 3;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;

	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;
}
