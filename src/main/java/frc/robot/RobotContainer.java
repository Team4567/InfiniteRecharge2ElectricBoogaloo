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
import edu.wpi.first.wpilibj.Joystick.AxisType;
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
import edu.wpi.first.vision.VisionThread;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Controls.BobController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.Gear;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  public final BallIn index;
  public final Misc misc;
  public final AdvShooter shoot;
  // Air Compressor
  public final Compressor compressor;
  
  public VisionThread visionThread;
  private Object imgLock = new Object();
  public double centerX, area;


  // Until we find something to put here, make it null.
  private Command m_autoCommand = null;
  

  // NetworkTables are a table of values that communicate between the laptop, robot, and other devices (Ex. Vision Camera)
  // Within the instance there are tables, and within the tables there are entries
  // The Limelight camera pre-defines its tables and entries
  // See https://docs.limelightvision.io/en/latest/getting_started.html#programming
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelight, vision;
  public NetworkTableEntry tx, ty, targetVisible, kPAlign, centerBall, maxA, centerA;
  
  // Constants for Limelight Calculations
  double angleLimelight = 0;
  double heightLimelight = 0;
  // Center of target
  double heightTarget = 89.75;
  double ballCenter = 3.5;
  double blink = 0.71;
  int stage = 1;


  // Define our controller in USB Port 0 (Port # shown in Driver Station)
  BobController controller = new BobController(0);

  // Presents a chooser for the driver to pick which auton to run out of a list
  SendableChooser<AutoPath> chooser;
  SendableChooser<Driver> driveChoose;

  // Boolean for testing driving the trajectories in reverse
  boolean reverseAuto = false;


  public enum Driver{
    MattGTA, MattNorm, Joe, Paul, Music
  }

  public enum AutoPath{
    test,ball,barrel,slalom,bounce
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Initiate our drivetrain, defaulting to low gear
    drive = new Drivetrain();
    drive.setGear(Drivetrain.Gear.HighGear);
    
    // Misc items, such as RGB Lights, PDP
    misc = new Misc();

    // Intake subsystem
    intake = new Intake();

    // Pulley and Brush
    index = new BallIn();

    shoot = new AdvShooter();

    // Setup Compressor
    compressor = new Compressor( Constants.kCANPCMA );
    compressor.setClosedLoopControl( true );
    compressor.stop();

    // Define all Limelight NetworkTables Tables and Entries
    limelight = inst.getTable( "limelight" );
    tx = limelight.getEntry( "tx" );
    ty = limelight.getEntry( "ty" );
    targetVisible = limelight.getEntry( "tv" );

    vision = inst.getTable( "Vision" );
    centerBall = vision.getEntry( "Center Ball?");
    centerA = vision.getEntry( "Center Ball A" );
    maxA = vision.getEntry( "Max Ball A" );



    // Define chooser, followed by available options
    chooser = new SendableChooser<AutoPath>();
    chooser.addOption("Test", AutoPath.test );
    chooser.addOption("Ball", AutoPath.ball );
    chooser.addOption("Barrel", AutoPath.barrel );
    chooser.addOption("Slalom", AutoPath.slalom );
    chooser.addOption("Bounce", AutoPath.bounce );

    driveChoose = new SendableChooser<Driver>();
    driveChoose.addOption("Joe", Driver.Joe );
    driveChoose.addOption("MattGTA", Driver.MattGTA );
    driveChoose.addOption("MattNorm", Driver.MattNorm );
    driveChoose.addOption("Paul", Driver.Paul );
    driveChoose.addOption("Music", Driver.Music );

    // None atm
    // chooser.addOption("Test", new SequentialCommandGroup( new TurnAngle( drive, 0
    // ), new DriveStraight( drive, 0 ) ) );
    // SmartDashboard.putData( "Auto Command", chooser );

    // Emergency button to reset commands, interrupting all active commands.
    SmartDashboard.putData("Reset Cmds",
      new InstantCommand( 
        () -> System.out.println( "All Subsystems Reset!" ), 
        drive, misc, intake, shoot, index
      )
    );

    SmartDashboard.putData( "Select Driver", driveChoose );
    SmartDashboard.putData( "Auto", chooser );

    SmartDashboard.putData("New Controls",
      new InstantCommand( 
        () -> {
          configureButtonBindings( driveChoose.getSelected() );
          System.out.println( "Controls Reset!" );

        }
      )
    );

    // Configure the button bindings
    configureButtonBindings( Driver.Joe );
    new Thread(() -> {
      
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      
      visionThread = new VisionThread( camera, new BallPipelineOpti(), pipeline -> {
      boolean t_centerBall = false;
      double t_maxA = -1;
      double t_centerArea = -1;
      ArrayList<MatOfPoint> outs = pipeline.filterContoursOutput();
          if( !outs.isEmpty() ){
            for( int i = 0; i < outs.size(); i++ ){
              Rect r = Imgproc.boundingRect( pipeline.filterContoursOutput().get( i ) );
              synchronized( imgLock ){
                centerX = r.x + (r.width/2);
                area = r.area();
              }
              if( area > t_maxA ){
                t_maxA = area;
              }
              if( Math.abs( centerX - 320 ) < 20 ){
                t_centerBall = true;
                t_centerArea = area;
              }
            }
            centerBall.setBoolean( t_centerBall );
            maxA.setDouble( t_maxA );
            centerA.setDouble( t_centerArea );

          }
          //output = pipeline.blurOutput();
      });
      Mat source = new Mat();
      Mat output = new Mat();
      visionThread.start();
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

      

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }

        //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        
        outputStream.putFrame( source );
      }
    }).start();

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
  private void configureButtonBindings( Driver d ) {
    switch( d ){
      case Joe:
        // Change something in TeleOp
        drive.setDefaultCommand(
          new RunCommand( 
            () -> drive.drive( controller::getLeftStickY, () -> -controller.getLeftStickX() )
            , drive 
          ) 
        );

        controller.startButton.whenPressed( new InstantCommand( drive::toggleGear ) );

        index.setDefaultCommand( 
          new RunCommand( 
            () -> index.pulleyPower( controller::getRightStickY )
            , index 
          ) 
        );

        intake.setDefaultCommand(
          new RunCommand(
            () -> intake.inPower( () -> controller.getRightTrigger()-controller.getLeftTrigger() )
            , intake
          )
        );

        controller.rightBumper.whenPressed( new InstantCommand( intake::liftToggle ) );

        misc.setDefaultCommand( 
          new RunCommand( 
            () -> misc.lightSet( () -> blink )
            , misc
          )
        );
 
        controller.bButton.whenPressed( new InstantCommand( shoot::toggle, shoot ) );
           
    
        controller.yButton.whenPressed( ()->{
          stage++;
          if( stage > 4 ){
            stage = 4;
          }
          switch( stage ){
            case 1:
              blink = 0.71;
              shoot.setSetpoint(86);
            break;
            case 2:
              blink = 0.67;
              shoot.setSetpoint(100);
            break;
            case 3:
              blink = 0.85;
              shoot.setSetpoint(120);
            break;
            case 4:
              blink = 0.61;
              shoot.setSetpoint(160);
            break;
          }
        });
        controller.aButton.whenPressed( ()->{
          stage--;
          if( stage < 1 ){
            stage = 1;
          }
          switch( stage ){
            case 1:
              blink = 0.71;
              shoot.setSetpoint(5000);
            break;
            case 2:
              blink = 0.67;
              shoot.setSetpoint(5000);
            break;
            case 3:
              blink = 0.85;
              shoot.setSetpoint(5000);
            break;
            case 4:
              blink = 0.61;
              shoot.setSetpoint(5000);
            break;
          }
        });
        controller.xButton.whenPressed( ()->{
          if( compressor.enabled() ){
            compressor.stop();
          }else{
            compressor.start();
          }
        });

        controller.dPadUp.whenHeld( new TrajectoryCommand( 
          new Pose2d( 0, 0, new Rotation2d( 0 ) ), 
          List.of(
            new Translation2d( 1, 1 )
           ), 
          new Pose2d( 2, 2, new Rotation2d( tx.getDouble(0) ) ),  false, drive ) 
          , true
        );

      break;
      case MattGTA:
        drive.setDefaultCommand( 
          new RunCommand(
            () -> drive.drive( () -> controller.getRightTrigger() - controller.getLeftTrigger(), () -> -controller.getLeftStickX() )
            , drive
          )
        );

        controller.dPadRight.whenPressed( new InstantCommand( drive::toggleGear ) );

        intake.setDefaultCommand( 
          new RunCommand(
              () -> intake.inPower( 0 )
            , intake
            )
        );

        controller.rightBumper.whenActive( new RunCommand( () -> intake.inPower( 1 ), intake ) );
        controller.leftBumper.whenActive( new RunCommand( () -> intake.inPower( -1 ), intake ) );
        controller.startButton.whenPressed( new InstantCommand( intake::liftToggle ) );

        controller.dPadUp.whenActive( new RunCommand( () -> index.pulleyPower( 1 ), index ) );
        controller.dPadDown.whenActive( new RunCommand( () -> index.pulleyPower( -1 ), index ) );

        controller.aButton.whenPressed( new InstantCommand( shoot::toggle, shoot ) );

      break;
      case MattNorm:
        drive.setDefaultCommand(
          new RunCommand( 
            () -> drive.drive( controller::getLeftStickY, () -> -controller.getLeftStickX() )
            , drive 
          ) 
        );
        controller.xButton.whenPressed( new InstantCommand( () -> drive.setGear( Gear.LowGear ) ) ); 
        controller.bButton.whenPressed( new InstantCommand( () -> drive.setGear( Gear.HighGear ) ) ); 

        controller.aButton.whenPressed( new InstantCommand( shoot::toggle, shoot ) );

        intake.setDefaultCommand(
          new RunCommand(
            () -> intake.inPower( () -> controller.getRightTrigger()-controller.getLeftTrigger() )
            , intake
          )
        );
        controller.startButton.whenPressed( new InstantCommand( intake::liftToggle ) );

        controller.rightBumper.whileHeld( new RunCommand( () -> index.pulleyPower( 1 ), index ) );
        controller.leftBumper.whileHeld( new RunCommand( () -> index.pulleyPower( -1 ), index ) );

      break;
      case Paul:
        drive.setDefaultCommand(
          new RunCommand( 
            () -> drive.drive( controller::getLeftStickY, () -> -controller.getLeftStickX() )
            , drive 
          ) 
        );
        controller.leftBumper.whenPressed( drive::toggleGear );
        
        controller.yButton.whileHeld( () -> intake.inPower( 1 ), intake );
        controller.xButton.whileHeld( () -> intake.inPower( -1 ), intake );

        controller.aButton.whileHeld( () -> index.pulleyPower( 1 ), index );
        controller.bButton.whileHeld( () -> index.pulleyPower( -1 ), index );

        controller.dPadDown.whenPressed( new InstantCommand( intake::liftToggle ) );

        controller.rightBumper.whenPressed( new InstantCommand( shoot::toggle ) );

      break;

    }

    /*
    controller.aButton.whenPressed( new InstantCommand( () -> shoot.enable(), shoot ) );
    controller.bButton.whenPressed( new InstantCommand( () -> shoot.
    disable(), shoot ) );
    controller.xButton.whenPressed( new InstantCommand( () -> shoot.setSetpoint( rpm.getDouble(85) ) ) );
    controller.startButton.whenPressed( 
      new InstantCommand(
        () -> intake.liftToggle( false )
      )
    );

    controller.selectButton.whenPressed( 
      new InstantCommand(
        () -> intake.liftToggle( true )
      )
    );
    controller.rightBumper.whenPressed( new InstantCommand( () -> drive.setGear( Drivetrain.Gear.HighGear ) ) );
    controller.leftBumper.whenPressed( new InstantCommand( () -> drive.setGear( Drivetrain.Gear.LowGear ) ) );
    */

    /*
     * Available Inputs
     * No controls are bound to these inputs
     * 
     * Left and Right Stick Click
     * Right Stick X
     * X Y B
     * Select
     * DPad
     * 
     */
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
    // Grab If we want to run this in reverse from the Shuffleboard, default to false
    reverseAuto = SmartDashboard.getBoolean( "Reverse", false );
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    TrajectoryConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward( Constants.ksDrive,
                                       Constants.kvDrive,
                                       Constants.kaDrive),
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
        new SimpleMotorFeedforward( Constants.ksDrive,
                                   Constants.kvDrive,
                                   Constants.kaDrive ),
        Constants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController( Constants.kPDrive, 0, Constants.kDDrive ),
        new PIDController( Constants.kPDrive, 0, Constants.kDDrive ),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive
    );

    // Reset odometry to the starting pose of the trajectory.
    //drive.resetOdometry( exampleTrajectory.getInitialPose() );

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen( () -> drive.tankDriveVolts( 0, 0 ) ) bn ;
    AutoPath selected = chooser.getSelected();
    if( selected == AutoPath.ball ){

    }else if( selected == AutoPath.barrel ){
      return new TrajectoryCommand( 
        new Pose2d( 0, 0, new Rotation2d( 0 ) ), 
        List.of(
          new Translation2d( 9, 0 ),
          new Translation2d( 11.5, -2.5 ),
          new Translation2d( 9, -5 ),
          new Translation2d( 7.5, -2.5 ),
          new Translation2d( 9, 0 ),
          new Translation2d( 16.5, 0 ),
          new Translation2d( 19, 2.5 ),
          new Translation2d( 16, 5 ),
          new Translation2d( 13.5, 2.5 ),
          new Translation2d( 16.5, 0 ),
          new Translation2d( 21.5, -5 ),
          new Translation2d( 24, -2.5 ),
          new Translation2d( 21.5, 0 )
        ), 
        new Pose2d( 0, 0, new Rotation2d( 0 ) ), 
        false, 
        drive
      ).get();
    }else if( selected == AutoPath.slalom ){
      return new TrajectoryCommand( 
        new Pose2d( 0, 0, new Rotation2d( 0 ) ), 
        List.of(
          new Translation2d( 4, 2.5 ),
          new Translation2d( 6.5, 5 ),
          //new Translation2d( 11.5, 7 ),
          new Translation2d( 16.5, 5 ),
          new Translation2d( 19, 2.5 ),
          new Translation2d( 21.5, 0 ),
          new Translation2d( 24, 2.5 ),
          new Translation2d( 21.5, 5 ),
          new Translation2d( 19, 2.5 ),
          new Translation2d( 16.5, 0 ),
          //new Translation2d( 11.5, -2 ),
          new Translation2d( 6.5, 0 ),
          new Translation2d( 4, 2.5 )
        ), 
        new Pose2d( -2.5, 5, new Rotation2d( 0 ) ), 
        false, 
        drive
      ).get();
    }else if( selected == AutoPath.bounce ){
      TrajectoryCommand a = new TrajectoryCommand( 
        new Pose2d( 0, 0, new Rotation2d( 0 ) ), 
        List.of(
          new Translation2d( 4, 2.5 )
        ), 
        new Pose2d( 4, 5, new Rotation2d( Math.PI/2 ) ), 
        false, 
        drive
      );
      TrajectoryCommand b = new TrajectoryCommand( 
        new Pose2d( 4, 5, new Rotation2d( Math.PI/2 ) ), 
        List.of(
          new Translation2d( 6.5, -2.5 ),
          new Translation2d( 9, -5 ),
          new Translation2d( 11.5, -2.5 )
        ), 
        new Pose2d( 11.5, 5, new Rotation2d( 3*Math.PI/2 ) ), 
        true, 
        drive
      );
      TrajectoryCommand c = new TrajectoryCommand( 
        new Pose2d( 11.5, 5, new Rotation2d( 3*Math.PI/2 ) ), 
        List.of(
          new Translation2d( 11.5, -2.5 ),
          new Translation2d( 15.25, -5 ),
          new Translation2d( 19, -2.5 )
        ), 
        new Pose2d( 19, 5, new Rotation2d( Math.PI/2 ) ), 
        false, 
        drive
      );
      TrajectoryCommand d = new TrajectoryCommand( 
        new Pose2d( 19, 5, new Rotation2d( Math.PI/2 ) ), 
        List.of(
          new Translation2d( 19, 2.5 )
        ), 
        new Pose2d( 24, 0, new Rotation2d( Math.PI ) ), 
        false, 
        drive
      );
      return new SequentialCommandGroup( a.get(), b.get(), c.get(), d.get() );
    }
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
    
  }*/
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward( Constants.ksDrive,
                                        Constants.kvDrive,
                                        Constants.kaDrive),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksDrive,
                                   Constants.kvDrive,
                                   Constants.kaDrive),
        Constants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(Constants.kPDrive, 0, 0),
        new PIDController(Constants.kPDrive, 0, 0),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive
    );

    // Reset odometry to the starting pose of the trajectory.
    //drive.m_gyro.setYaw(0);
    drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
  }
}

