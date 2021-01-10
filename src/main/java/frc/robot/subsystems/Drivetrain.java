package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.IMU;

public class Drivetrain extends SubsystemBase {

	public static enum Gear{
		HighGear,
		LowGear
	}
	// double : 1
	final double lowGearRatio  = 15.32;
	final double highGearRatio = 7.08;
	final double cpr = 2048;
	final double wheelCirc = 6*Math.PI;
	public double inchesToUnits = 1;
	public WPI_TalonFX leftMaster = new WPI_TalonFX( Constants.kCANLMaster ), 
		leftSlave = new WPI_TalonFX( Constants.kCANLSlave ), 
		rightMaster = new WPI_TalonFX( Constants.kCANRMaster ), 
		rightSlave = new WPI_TalonFX( Constants.kCANRSlave );
	public DoubleSolenoid gearShift = new DoubleSolenoid( Constants.kCANPCMA, Constants.kPCMLGearboxIn, Constants.kPCMLGearboxOut );;
	public Gear gear, prevGear;
	// The motors on the left side of the drive.


  	private final SpeedControllerGroup m_leftMotors =
    	new SpeedControllerGroup(leftMaster,
                                 leftSlave);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
  	  new SpeedControllerGroup(rightMaster,
  							   rightSlave);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive( m_leftMotors, m_rightMotors );


				  
  
  // The gyro sensor
  private final IMU m_gyro = new IMU( Constants.kCANIMU );

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public Drivetrain() {
    // Sets the distance per pulse for the encoders
    //m_leftEncoder.setDistancePerPulse( Constants.kEncoderDistancePerPulse );
	//m_rightEncoder.setDistancePerPulse( Constants.kEncoderDistancePerPulse );
	leftMaster.configSelectedFeedbackSensor( TalonFXFeedbackDevice.IntegratedSensor, 0, 0 );
	// 2048 CPR
	// 512 PPR (Motor Shaft)
	// 7843.84 PPR (Output Shaft)
	// 0.00012748857 RPP (Output Shaft)
	// 0.00240310306 Inches Per Pulse (Wheel)
	leftMaster.configSelectedFeedbackCoefficient( ( 6 * Math.PI ) / ( 2048 * lowGearRatio * 39.37 ) );
	rightMaster.configSelectedFeedbackCoefficient( ( 6 * Math.PI ) / ( 2048 * lowGearRatio * 39.37 ) );
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry( m_gyro.getRotation2d() );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    //m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
    //                  m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( leftMaster.getSelectedSensorVelocity()*10, rightMaster.getSelectedSensorVelocity()*10 );
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void drive( DoubleSupplier y, DoubleSupplier x ){
	  arcadeDrive( y.getAsDouble(), x.getAsDouble() );
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    //m_leftEncoder.reset();
    //m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ( leftMaster.getSelectedSensorPosition() + rightMaster.getSelectedSensorPosition() ) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public WPI_TalonFX getLeftMaster() {
    return leftMaster;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public WPI_TalonFX getRightMaster() {
    return rightMaster;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput( double maxOutput ) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getYawRate();
  }

  public void setGear( Gear g ){
	  prevGear = gear;
	  gear = g;
  }

}