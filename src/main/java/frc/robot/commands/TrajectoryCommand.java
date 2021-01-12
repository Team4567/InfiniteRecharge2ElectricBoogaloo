// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryCommand extends RamseteCommand {
  Pose2d start, end;
  List<Translation2d> list;
  boolean reverseAuto;
  Trajectory t;
  Drivetrain drive;
  
  // A
  public TrajectoryCommand ( Trajectory traj, Drivetrain drive ){
    /*
        Ramsete Requirements

        Trajectory trajectory, 
        Supplier<Pose2d> pose, 
        RamseteController controller, 
        SimpleMotorFeedforward feedforward, 
        DifferentialDriveKinematics kinematics, 
        Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, 
        PIDController leftController, 
        PIDController rightController, 
        BiConsumer<Double, Double> outputVolts, 
        Subsystem... requirements
    */
    // Since RamseteCommand is the "parent" class (TrajectoryCommand extends RamseteCommand), we must call super() to give Ramsete the parameters it needs to exist. Seen above.
    super(
      // Trajectory
      traj,

      // Pose
      drive::getPose,

      // RamseteController
      new RamseteController( Constants.kRamseteB, Constants.kRamseteZeta ),

      // FeedForward
      new SimpleMotorFeedforward( Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter ),

      // DriveKinematics                             
      Constants.kDriveKinematics,

      // Wheel Speeds
      drive::getWheelSpeeds,

      // Left PID
      new PIDController( Constants.kPDriveVel, 0, Constants.kDDriveVel ),

      // Right PID
      new PIDController( Constants.kPDriveVel, 0, Constants.kDDriveVel ),

      // Output Volts
      drive::tankDriveVolts,

      // Subsystem
      drive

    );

    // Save the trajectory for later
    t = traj;
  }


  // B
  public TrajectoryCommand( Pose2d start, List<Translation2d> list, Pose2d end, boolean reverseAuto, Drivetrain drive ) {
    // this() takes the parameters of a different constructor (A) and uses it with the info its given
    // A doesnt want a Pose2d, List<Translation2d>, Pose2d and boolean, it wants a trajectory. So, you tell it to use your info in the proper format.
    this(
      // Trajectory
      TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        start,
        // Pass through these two interior waypoints, making an 's' curve path
        list,
        // End 3 meters straight ahead of where we started, facing forward
        end,
        // Pass config
        new TrajectoryConfig( Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared )
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics( Constants.kDriveKinematics )
            // Apply the voltage constraint 
            .addConstraint( 
                new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward( Constants.ksVolts,
                                         Constants.kvVoltSecondsPerMeter,
                                         Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                10 ) 
              )
            
            .setReversed( reverseAuto )
      ),

      // Subsystem
      drive

    );
    this.drive = drive;
  }
 
  // C
  public TrajectoryCommand( ControlVectorList list, boolean reverseAuto, Drivetrain drive ) {
    // this() takes the parameters of a different constructor (A) and uses it with the info its given
    // A doesnt want a ControlVectorList and boolean, it wants a trajectory. So, you tell it to use your info in the proper format.
    this(
      // Trajectory
      TrajectoryGenerator.generateTrajectory(
        // Pass through these two interior waypoints, making an 's' curve path
        list,
        // Pass config
        new TrajectoryConfig( Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared )
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics( Constants.kDriveKinematics )
            // Apply the voltage constraint 
            .addConstraint( 
                new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward( Constants.ksVolts,
                                         Constants.kvVoltSecondsPerMeter,
                                         Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                10 ) 
              )
            
            .setReversed( reverseAuto )
      ),

      // Subsystem
      drive

    );
    this.drive = drive;
  }

  public SequentialCommandGroup get(){
    // Set the robot's "believed position (pose)" to be the starting position of the trajectory
    drive.resetOdometry( t.getInitialPose() );

    // this refers to the object you are calling this from
    // If I have TrajectoryCommand c, and I do c.get()
    // Then "this" is the same this as c
    // The andThen command adds a 2nd action when the command is done
    // Do c, and Then do this also
    // This LAMBDA (Notice the () -> ) tells the robot to stop giving voltage to the motors, stopping.
    return this.andThen( () -> drive.tankDriveVolts( 0, 0 ) );
  }

}
