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
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryCommand extends RamseteCommand {
  
  // This is an attempt to make something that will make full trajectory commands without the lines of code in Robot Container
  public TrajectoryCommand( Pose2d start, List<Translation2d> list, Pose2d end, boolean reverseAuto, Drivetrain drive ) {
    /*
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
    super(
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
  }

}
