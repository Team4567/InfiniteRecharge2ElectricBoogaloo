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
  // public final ControlPanel control;
  public final Misc misc;

  public final Compressor compressor;
  private Command m_autoCommand = null;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table, limelight;
  NetworkTableEntry tx, ty, targetVisible, rpm, blink, kPAlign;

  BobController controller = new BobController(0);

  double angleLimelight = 0;
  double heightLimelight = 0;
  // Center of target
  double heightTarget = 89.75;

  SendableChooser<CommandGroupBase> chooser;
  DriverStation.Alliance alliance;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = new Drivetrain();
    drive.setGear(Drivetrain.Gear.LowGear);
    // control = new ControlPanel();
    misc = new Misc();
    intake = new Intake();
    compressor = new Compressor(Constants.kCANPCMA);

    compressor.setClosedLoopControl(true);
    compressor.start();

    limelight = inst.getTable("limelight");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    targetVisible = limelight.getEntry("tv");

    table = inst.getTable("Table");

    rpm = table.getEntry("RPM");
    rpm.setDouble(-8000);

    blink = table.getEntry("Blink");
    blink.setDouble(0);

    kPAlign = table.getEntry("kPAlign");
    kPAlign.setDouble(0);

    // m_autoCommand = new DriveStraight( drive, 120 );

    chooser = new SendableChooser<CommandGroupBase>();
    
    // chooser.addOption("Test", new SequentialCommandGroup( new TurnAngle( drive, 0
    // ), new DriveStraight( drive, 0 ) ) );
    // SmartDashboard.putData( "Auto Command", chooser );

    SmartDashboard.putData("Reset Cmds",
        new InstantCommand(() -> System.out.println("All Subsystems Reset!"), drive, misc, intake));
    // Configure the button bindings
    configureButtonBindings();
  }

  public double getTargetRPM() {
    return rpm.getDouble(0);
  }

  public boolean haveTarget() {
    return targetVisible.getDouble(0) == 1;
  }

  public double getDistanceToTarget() {
    return haveTarget() ? (heightTarget - heightLimelight) / Math.tan(angleLimelight + ty.getDouble(0)) : 0;
  }

  public double getAngleToTarget() {
    return haveTarget() ? tx.getDouble(0) : 0;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        new RunCommand(() -> drive.drive(controller::getLeftStickY, controller::getLeftStickX), drive ) );
    // control.setDefaultCommand( new RunCommand( () -> control::stop, control ) );
    intake.setDefaultCommand(new RunCommand(() -> intake.control(() -> controller.getRightStickY() * 0.5,
        () -> controller.getRightTrigger() - controller.getLeftTrigger()), intake));
    misc.setDefaultCommand(new RunCommand(() -> misc.lightDefault(() -> targetVisible.getDouble(0) == 1), misc));

    // controller.dPadLeft.whenHeld( new RunCommand( () -> control.control( -0.75 ),
    // control ) );
    // controller.dPadRight.whenHeld( new RunCommand( () -> control.control( 0.75 ),
    // control ) );
    // controller.selectButton.whenPressed( new RunCommand( () ->
    // control.togglePiston() ) );

    controller.rightBumper.whenPressed(new InstantCommand(() -> drive.setGear(Drivetrain.Gear.HighGear)));
    controller.leftBumper.whenPressed(new InstantCommand(() -> drive.setGear(Drivetrain.Gear.LowGear)));

    /*
     * Available Inputs
     * 
     * Right Stick X A Y X B Start Select DPad L/R
     * 
     */
    
      // These are meme bindings
      //controller.startButton.whenHeld( new RunCommand( () -> drive.drive( controller::getLeftStickY, () -> getAngleToTarget() * kPAlign.getDouble( 0 ) ), drive ) );
      //controller.selectButton.whenHeld( new RunCommand( misc::lightToControl, misc ) );
      
      //controller.startButton.whenHeld( new RunCommand( control::setToColor, control ) );
        //.whenReleased( new InstantCommand( control::stop, control ) );
  }

  public void setAlliance( DriverStation.Alliance allaince ){
    this.alliance = alliance;
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward( Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10 );

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig( Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared )
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics( Constants.kDriveKinematics )
            // Apply the voltage constraint 
            .addConstraint( autoVoltageConstraint );

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d( 0, 0, new Rotation2d( 0 ) ),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d( 1, 1 ),
            new Translation2d( 2, -1 )
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d( 3, 0, new Rotation2d( 0 ) ),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drive::getPose,
        new RamseteController( Constants.kRamseteB, Constants.kRamseteZeta ),
        new SimpleMotorFeedforward( Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter ),
        Constants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController( Constants.kPDriveVel, 0, 0 ),
        new PIDController( Constants.kPDriveVel, 0, 0 ),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive
    );

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry( exampleTrajectory.getInitialPose() );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen( () -> drive.tankDriveVolts(0, 0) );
  }
}
