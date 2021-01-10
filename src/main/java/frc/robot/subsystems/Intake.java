/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  VictorSPX in;
  TalonSRX flip;
  /**
   * Creates a new Intake.
   */
  public Intake() {
    in = new VictorSPX( Constants.kCANIntake );
    in.configFactoryDefault();
    in.setNeutralMode( NeutralMode.Brake );
    in.setInverted( false );
    in.configPeakOutputForward( +1.0 );
    in.configPeakOutputReverse( -1.0 );
    in.configNeutralDeadband( 0.1 );
    flip = new TalonSRX( Constants.kCANFlip );
    flip.configFactoryDefault();
    flip.setNeutralMode( NeutralMode.Brake );
    flip.setInverted( false );
    flip.configPeakOutputForward( +1.0 );
    flip.configPeakOutputReverse( -1.0 );
    flip.configNeutralDeadband( 0.1 );
    flip.configForwardLimitSwitchSource( LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    flip.configReverseLimitSwitchSource( LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);

  }

  public void controlIntake( double value ){
    in.set( ControlMode.PercentOutput, value );
  }
  public void controlFlip( double value ){
    flip.set( ControlMode.PercentOutput, value );
  }

  public void controlIntake( DoubleSupplier value ){
    controlIntake( value.getAsDouble() );
  }
  public void controlFlip( DoubleSupplier value ){
    controlFlip( value.getAsDouble() );
  }

  public void control( DoubleSupplier flip, DoubleSupplier in ){
    controlIntake( in.getAsDouble() );
    controlFlip( flip.getAsDouble() );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
