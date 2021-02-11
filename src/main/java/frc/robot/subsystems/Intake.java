// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  DoubleSolenoid lift;
  VictorSPX spin;
  /** Creates a new Intake. */
  public Intake() {
    spin = new VictorSPX( Constants.kCANIntake );
    lift = new DoubleSolenoid( Constants.kPCMLiftIn, Constants.kPCMLiftOut, Constants.kCANPCMA );
  }

  public void liftToggle( boolean on ){
    lift.set( on ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse );
  }

  public void inToggle( boolean on ){
    spin.set( VictorSPXControlMode.PercentOutput, on ? 1 : 0 );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
