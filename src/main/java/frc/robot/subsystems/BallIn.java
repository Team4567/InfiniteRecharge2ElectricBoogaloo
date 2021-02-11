// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallIn extends SubsystemBase {
  VictorSPX pulley, brush;
  /** Creates a new BallIn. */
  public BallIn() {
    pulley = new VictorSPX( Constants.kCANPulley );
    pulley.configFactoryDefault();
    pulley.setNeutralMode( NeutralMode.Coast );
    pulley.setInverted( false );
    
    brush = new VictorSPX( Constants.kCANBrush );
    brush.configFactoryDefault();
    brush.setNeutralMode( NeutralMode.Coast );
    brush.setInverted( false );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void brushToggle( boolean on ){
    brush.set( ControlMode.PercentOutput, on ? 1 : 0 );
  }

  public void pulleyToggle( boolean on ){
    pulley.set( ControlMode.PercentOutput, on ? 1 : 0 );
  }
}