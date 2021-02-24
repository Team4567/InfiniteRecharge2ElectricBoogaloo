// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BasicShooter extends SubsystemBase {
  TalonSRX shooter;
  double constant = 600/4096;
  /** Creates a new BasicShooter. */
  public BasicShooter(){
    shooter = new TalonSRX( Constants.kCANShooter );
    shooter.configFactoryDefault();
    shooter.setNeutralMode( NeutralMode.Coast );
    shooter.setInverted( true );
    shooter.configPeakOutputForward( +1.0 );
    shooter.configPeakOutputReverse( -1.0 );
    shooter.configSelectedFeedbackSensor( TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0 );
    shooter.configSelectedFeedbackCoefficient( constant );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower( double power ){
    shooter.set( ControlMode.PercentOutput, power );
  }

  public void setPower( DoubleSupplier power ){
    setPower( power.getAsDouble() );
  }
}
