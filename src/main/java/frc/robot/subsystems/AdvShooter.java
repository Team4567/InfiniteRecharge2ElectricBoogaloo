// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class AdvShooter extends PIDSubsystem {
  WPI_TalonSRX shooter;
  // 10/4096
  double constant = 0.00244141;
  double measurement;
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward( Constants.ksShooter,
                                  Constants.kvShooter,
                                  Constants.kaShooter 
                                );
  /** Creates a new AdvShooter. */
  public AdvShooter() {
    super( new PIDController( Constants.kPShooter, 0, Constants.kDShooter ) );
    getController().setTolerance( Constants.kShooterToleranceRPS );
    setSetpoint(85);
    shooter = new WPI_TalonSRX( Constants.kCANShooter );
    shooter.configFactoryDefault();
    shooter.setNeutralMode( NeutralMode.Coast );
    shooter.setInverted( true );
    //shooter.configPeakOutputForward( +1.0 );
    //shooter.configPeakOutputReverse( -1.0 );
    shooter.configSelectedFeedbackSensor( TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0 );
    shooter.configSelectedFeedbackCoefficient( 1 );
  }

  @Override
  public void useOutput( double output, double setpoint ) {
    // Use the output here
    double ff = m_shooterFeedforward.calculate( setpoint );
    double value = output + ff;
    shooter.setVoltage( value );
    
    SmartDashboard.putNumber( "Use Out", value );
  }

  public void setSetpoint( DoubleSupplier setpoint ) {
    // TODO Auto-generated method stub
    setSetpoint( setpoint.getAsDouble() / 60 );
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    double v = shooter.getSelectedSensorVelocity() * constant;
    measurement = v;
    return v;
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
    double s = getSetpoint();
    double e = m_controller.getPositionError();
    SmartDashboard.putNumber( "Target", s );
    SmartDashboard.putNumber( "Measurement", measurement );
    SmartDashboard.putNumber( "Error", e );
    SmartDashboard.putNumber( "Raw Target", s / constant );
    SmartDashboard.putNumber( "Raw Measurement", measurement / constant );
    SmartDashboard.putNumber( "Raw Error", e / constant );
    SmartDashboard.putNumber( "Motor Out", -shooter.getMotorOutputVoltage() );
    SmartDashboard.putBoolean("m_enabled", isEnabled() );
  }
}
