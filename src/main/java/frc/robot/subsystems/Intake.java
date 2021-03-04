// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  DoubleSolenoid liftL, liftR;
  VictorSPX spin;
  boolean up;
  /** Creates a new Intake. */
  public Intake() {
    spin = new VictorSPX( Constants.kCANIntake );
    spin.setInverted( true );
    liftL = new DoubleSolenoid( Constants.kCANPCMA, Constants.kPCMLiftInL, Constants.kPCMLiftOutL );
    liftR = new DoubleSolenoid( Constants.kCANPCMA, Constants.kPCMLiftInR, Constants.kPCMLiftOutR );
  }

  public void liftToggle( boolean on ){
    up = on;
  }

  public void inPower( double power ){
    spin.set( VictorSPXControlMode.PercentOutput, power );
  }

  public void liftToggle( BooleanSupplier on ){
    liftToggle( on.getAsBoolean() );
  }

  public void liftToggle(){
    up = !up;
  }

  public void inPower( DoubleSupplier power ){
    inPower( power.getAsDouble() );
  }

  public boolean getUp(){
    return liftL.get() == Value.kReverse;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
      liftL.set( up ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse );
      liftR.set( !up ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse );
  
  }
}
