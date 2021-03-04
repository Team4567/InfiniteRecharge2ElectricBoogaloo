/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Misc extends SubsystemBase {
  public Spark blinkin;
  public PowerDistributionPanel pdp;
  
  /**
   * Creates a new Misc.
   */
  public Misc() {
    blinkin = new Spark( 9 );
    pdp = new PowerDistributionPanel();
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putData( "PDP",pdp );
    
    
    // This method will be called once per scheduler run
  }

  public String getGameMessage(){
    String out = DriverStation.getInstance().getGameSpecificMessage();
    if( out.length() == 0 ){
      return "N/A";
    }else{
      return out;
    }
  }
  
  public void lightSet( double in ){
    blinkin.set( in );
  }
  public void lightSet( DoubleSupplier in ){
    lightSet( in.getAsDouble() );
  }

  public void lightDefault( BooleanSupplier visionTarget ){
    // Rainbow else green/white
    blinkin.set( visionTarget.getAsBoolean() ? -0.45 : 0.53 );
  }
}
