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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
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
  public DoubleSolenoid gearShiftL = new DoubleSolenoid( Constants.kCANPCMA, Constants.kPCMLGearboxIn, Constants.kPCMLGearboxOut );
  public DoubleSolenoid gearShiftR = new DoubleSolenoid( Constants.kCANPCMA, Constants.kPCMRGearboxIn, Constants.kPCMRGearboxOut );;
	public Gear gear, prevGear;
	// The motors on the left side of the drive.


  	private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup( leftMaster, leftSlave );

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup( rightMaster, rightSlave );

  // The robot's drive, gives access to simplified drive commands. AUTOMATICALLY INVERTS ONE OF THE SIDES
  private final DifferentialDrive m_drive = new DifferentialDrive( m_leftMotors, m_rightMotors );


				  
  
  // The gyro sensor, gives robots angle, acceleration, and turn rate.
  private final IMU m_gyro = new IMU( Constants.kCANIMU );

  // Odometry class for tracking robot pose
  // Odometry simply means tracking a robot using a variety of sensors to apporximate its real life position
  // We use odometry to follow paths to make sure we get to the right coordinates.
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public Drivetrain() {
    // Tell the leftMaster to reference its integrated encoder as its sensor
    leftMaster.configSelectedFeedbackSensor( TalonFXFeedbackDevice.IntegratedSensor, 0, 0 );
    // Tell the rightMaster to reference its integrated encoder as its sensor
    rightMaster.configSelectedFeedbackSensor( TalonFXFeedbackDevice.IntegratedSensor, 0, 0 );
	  // 2048 CPR
	  // 512 PPR (Motor Shaft)
	  // 7843.84 PPR (Output Shaft)
	  // 0.00012748857 RPP (Output Shaft)
	  // 0.00240310306 Inches Per Pulse (Wheel)
	  leftMaster.configSelectedFeedbackCoefficient( ( 6 * Math.PI ) / ( 2048 * lowGearRatio ) );
	  rightMaster.configSelectedFeedbackCoefficient( ( 6 * Math.PI ) / ( 2048 * lowGearRatio ) );
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry( m_gyro.getRotation2d() );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update( m_gyro.getRotation2d(), Units.feetToMeters( leftMaster.getSelectedSensorPosition() / 12 ),
      Units.feetToMeters( rightMaster.getSelectedSensorPosition() / 12 ) );

    // Puts data that we may need into the Shuffleboard to view it
    SmartDashboard.putData( "Drivetrain", m_drive );
    SmartDashboard.putData( "Gyro", m_gyro );
    SmartDashboard.putNumber( "Left Encoder", leftMaster.getSelectedSensorPosition() );
    SmartDashboard.putNumber( "Right Encoder", rightMaster.getSelectedSensorPosition() );
    SmartDashboard.putNumber( "Left Velocity", leftMaster.getSelectedSensorVelocity() * 10 );
    SmartDashboard.putNumber( "Right Velocity", rightMaster.getSelectedSensorVelocity() * 10 );

    if( gear != prevGear ){
      /*
        INLINE IF STATEMENT
        Imaginet the inside of the 2 sets below is a variable named v
        Instead of
        if( gear == Gear.HighGear ){
          v = kForward;
        }else{
          v = kReverse;
        }
        set(v);
        set(v);
        We put the if inside the parentheses with an inline if
        set( boolean ? true value : false value )
      */
      gearShiftL.set( gear == Gear.HighGear ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse );
      gearShiftR.set( gear == Gear.HighGear ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse );
      prevGear = gear;
    }
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
    leftMaster.setSelectedSensorPosition( 0 );
    rightMaster.setSelectedSensorPosition( 0 );
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ( leftMaster.getSelectedSensorPosition() + rightMaster.getSelectedSensorPosition() ) / 2;
  }


  // These are in case another function needs the encoder values, we can give it through the motor
  /**
   * Gets the left drive master motor
   *
   * @return the left drive master motor
   */
  public WPI_TalonFX getLeftMaster() {
    return leftMaster;
  }

  /**
   * Gets the right drive master motor
   *
   * @return the right drive master motor
   */
  public WPI_TalonFX getRightMaster() {
    return rightMaster;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained, Interval (0, 1]
   */
  public void setMaxOutput( double maxOutput ) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot. Wherever we are facing, that is now 0 degrees
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
	  gear = g;
  }

}