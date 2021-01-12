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
	 * Each gearbox has 2 motors. One is the master that does calculations, and the slave that follows whatever the master does
     */
	// CAN ID of Left Master
	public static final int kCANLMaster = 3;
	// CAN ID of Left Slave
	public static final int kCANLSlave = 0;
	// CAN ID of Right Master
	public static final int kCANRMaster = 2;
	// CAN ID of Right Slave
    public static final int kCANRSlave = 1;

	// CAN ID of Intake Motor
	public static final int kCANIntake = 0;
	// CAN ID of Flip Motor
	public static final int kCANFlip = 15;

	// CAN ID of IMU (Gyro)
	public static final int kCANIMU = 5;
	
	// CAN ID of Pneumatics Control Module A (Called A in case we have a 2nd)
	public static final int kCANPCMA = 10;
	
	// PCM Port Numbers for the Left Gearbox Shifter
	// This acts as a normal piston, but instead of being a long shaft it pushes the gears into different alignments for different ratios
	public static final int kPCMLGearboxIn = 0;
	public static final int kPCMLGearboxOut = 1;

	// PCM Port Numbers for the Left Gearbox Shifter
	// This acts as a normal piston, but instead of being a long shaft it pushes the gears into different alignments for different ratios
	public static final int kPCMRGearboxIn = 2;
	public static final int kPCMRGearboxOut = 3;
	
	/**
	 * This is a property of the Pigeon IMU, and should not be changed.
	 */
	public final static int kPigeonUnitsPerRotation = 8192;

    // Tuned Values from the FRC Characterization Sheet
	public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
	public static final double kPDriveVel = 8.5;
	public static final double kDDriveVel = 8.5;
	public static final double kTrackwidthMeters = 0.69;
	
	// Takes the track width and makes a Kinematics file for the trajectory calculator to understand
    public static final DifferentialDriveKinematics kDriveKinematics =
		new DifferentialDriveKinematics( kTrackwidthMeters );
		
	// Max Speed and Acceleration
	public static final double kMaxSpeedMetersPerSecond = 3;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;

	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
	// Ramsete Controller Converts Point Trajectories to Voltage output for the robots motors
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;
}
