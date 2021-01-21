/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Controls.BobController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// Pineapple

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain drive;
  public final Intake intake;
  public final Misc misc;

  // Air Compressor
  public final Compressor compressor;
  
  // Until we find something to put here, make it null.
  private Command m_autoCommand = null;

  // NetworkTables are a table of values that communicate between the laptop, robot, and other devices (Ex. Vision Camera)
  // Within the instance there are tables, and within the tables there are entries
  // The Limelight camera pre-defines its tables and entries
  // See https://docs.limelightvision.io/en/latest/getting_started.html#programming
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table, limelight;
  NetworkTableEntry tx, ty, targetVisible, rpm, blink, kPAlign;
  
  // Constants for Limelight Calculations
  double angleLimelight = 0;
  double heightLimelight = 0;
  // Center of target
  double heightTarget = 89.75;
  double ballCenter = 3.5;


  // Define our controller in USB Port 0 (Port # shown in Driver Station)
  BobController controller = new BobController(0);

  // Presents a chooser for the driver to pick which auton to run out of a list
  SendableChooser<CommandGroupBase> chooser;

  // Boolean for testing driving the trajectories in reverse
  boolean reverseAuto = false;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initiate our drivetrain, defaulting to low gear
    drive = new Drivetrain();
    drive.setGear(Drivetrain.Gear.LowGear);
    
    // Misc items, such as RGB Lights, PDP
    misc = new Misc();

    // Intake subsystem
    intake = new Intake();

    // Setup Compressor
    compressor = new Compressor(Constants.kCANPCMA);
    compressor.setClosedLoopControl(true);
    compressor.start();

    // Define all Limelight NetworkTables Tables and Entries
    limelight = inst.getTable("limelight");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    targetVisible = limelight.getEntry("tv");

    table = inst.getTable("Table");

    rpm = table.getEntry("RPM");
    // setDouble sets the value within the NetworkTable, as the form of a double (Could use setBoolean() for boolean, etc.)
    rpm.setDouble(-8000);

    blink = table.getEntry("Blink");
    blink.setDouble(0);

    kPAlign = table.getEntry("kPAlign");
    kPAlign.setDouble(0);

    // Define chooser, followed by available options
    chooser = new SendableChooser<CommandGroupBase>();
    
    // None atm
    // chooser.addOption("Test", new SequentialCommandGroup( new TurnAngle( drive, 0
    // ), new DriveStraight( drive, 0 ) ) );
    // SmartDashboard.putData( "Auto Command", chooser );

    // Emergency button to reset commands, interrupting all active commands.
    SmartDashboard.putData("Reset Cmds",
        new InstantCommand(() -> System.out.println("All Subsystems Reset!"), drive, misc, intake));

    // Configure the button bindings
    configureButtonBindings();
  }

  public double getTargetRPM() {
    // Get Double will get the value of the rpm entry, but if none can be found, default to the inside number (0)
    return rpm.getDouble(0);
  }

  public boolean haveTarget() {
    // When Limelight has a target lock, it displays the number 1.
    return targetVisible.getDouble(0) == 1;
  }

  // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public double getDistanceToTarget() {
    // Inline If Statement
    /*
      if( boolean b ){
        value a
      }else{
        value b
      }
      IS THE SAME AS
      b ? a : b
    */
    return haveTarget() ? (heightTarget - heightLimelight) / Math.tan(angleLimelight + ty.getDouble(0)) : 0;
  }

  // Get the horizontal angle we are from the target
  public double getAngleToTarget() {
    // Inline If Statement
    /*
      if( boolean b ){
        value a
      }else{
        value b
      }
      IS THE SAME AS
      b ? a : b
    */
    return haveTarget() ? tx.getDouble(0) : 0;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * 
   * A default comamnd will be active when there are no other commands to be ran in the list for that subsystem. 
   * So, when theres no more auto commands in teleop, it defualts to their controls
   */
  private void configureButtonBindings() {

    // Change something in TeleOp
    drive.setDefaultCommand(
        new RunCommand( () -> drive.drive( controller::getLeftStickY, controller::getLeftStickX ), drive ) );

    intake.setDefaultCommand(new RunCommand(() -> intake.control(() -> controller.getRightStickY() * 0.5,
        () -> controller.getRightTrigger() - controller.getLeftTrigger()), intake));

    misc.setDefaultCommand( new RunCommand(() -> misc.lightDefault( () -> targetVisible.getDouble(0) == 1 ), misc ) );

    controller.rightBumper.whenPressed( new InstantCommand( () -> drive.setGear( Drivetrain.Gear.HighGear ) ) );
    controller.leftBumper.whenPressed( new InstantCommand( () -> drive.setGear( Drivetrain.Gear.LowGear ) ) );

    /*
     * Available Inputs
     * No controls are bound to these inputs
     * 
     * Left and Right Stick Click
     * Right Stick X
     * X A Y B 
     * Start Select 
     * DPad
     * 
     */
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Grab If we want to run this in reverse from the Shuffleboard, default to false
    reverseAuto = SmartDashboard.getBoolean( "Reverse", false );
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    TrajectoryConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward( Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10 );

    // Create config for trajectory
    // Takes into consideration all parameters of our robot
    TrajectoryConfig config =
        new TrajectoryConfig( Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared )
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics( Constants.kDriveKinematics )
            // Apply the voltage constraint 
            .addConstraint( autoVoltageConstraint )
            // Determines forwards/reverse
            .setReversed( reverseAuto );

    // An example trajectory to follow.  All units in meters.
    // https://youtu.be/yVmJDOE3M2Y
    // https://youtu.be/FLn1bFqlkL0
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        // If going in reverse, we are actually facing 180 degrees (In the robot's mind)
        new Pose2d( 0, 0, new Rotation2d( reverseAuto ? Math.PI : 0 ) ),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d( 1, 1 ),
            new Translation2d( 2, -1 )
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d( 3, 0, new Rotation2d( reverseAuto ? Math.PI : 0 ) ),
        // Pass config
        config
    );

    // The actual command that generates the voltages needed for each part
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drive::getPose,
        new RamseteController( Constants.kRamseteB, Constants.kRamseteZeta ),
        new SimpleMotorFeedforward( Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter ),
        Constants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController( Constants.kPDriveVel, 0, Constants.kDDriveVel ),
        new PIDController( Constants.kPDriveVel, 0, Constants.kDDriveVel ),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive
    );

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry( exampleTrajectory.getInitialPose() );

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen( () -> drive.tankDriveVolts( 0, 0 ) );

    return new TrajectoryCommand( 
      new Pose2d( 0, 0, new Rotation2d( reverseAuto ? Math.PI : 0 ) ), 
      List.of(
        new Translation2d( 1, 1 ),
        new Translation2d( 2, -1 )
      ), 
      new Pose2d( 3, 0, new Rotation2d( reverseAuto ? Math.PI : 0 ) ), 
      reverseAuto, 
      drive
    ).get();

  }
}
