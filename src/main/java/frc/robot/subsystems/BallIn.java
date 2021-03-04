// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallIn extends SubsystemBase {
  TalonSRX pulley;
  /** Creates a new BallIn. */
  public BallIn() {
    pulley = new TalonSRX( Constants.kCANPulley );
    pulley.configFactoryDefault();
    pulley.setNeutralMode( NeutralMode.Brake );
    pulley.setInverted( true );
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void pulleyPower( double power ){
    pulley.set( ControlMode.PercentOutput, power );
  }

  public void pulleyPower( DoubleSupplier power ){
    pulleyPower( power.getAsDouble() );
  }
}
