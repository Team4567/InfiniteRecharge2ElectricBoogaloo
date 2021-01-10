/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
    blinkin = new Spark( 0 );
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
  
  public void lightToControl(){
    switch( getGameMessage() ){
      case "R":
        // Red
        blinkin.set( 0.61 );
        break;
      case "G":
        // Green
        blinkin.set( 0.77 );
        break;
      case "B":
        // Blue
        blinkin.set( 0.87 );
        break;
      case "Y":
        // Yellow
        blinkin.set( 0.69 );
        break;
      default:
        // Black
        blinkin.set( 0.99 );
        break; 
    }
  }

  public void lightDefault( BooleanSupplier visionTarget ){
    // Rainbow else green/white
    blinkin.set( visionTarget.getAsBoolean() ? -0.45 : 0.53 );
  }
}
