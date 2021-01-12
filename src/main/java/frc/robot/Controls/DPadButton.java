package frc.robot.Controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * In standard Xbox controllers, the D-Pad is not treated as 4 buttons, rather as like a compass
 * Up was 0, and it went clockwise.
 * With this, it turns the cardinal 4 values (0,90,180,270) and makes them booleans (true/false)
 */
public class DPadButton extends Button{
    // Takes the numeric input from the controller
    DoubleSupplier in;

    // The target number we want for a true value
    double dirNum;

    // Makes a more logical method of displaying which Direction we want this button to represent
    public enum Direction{
        UP,
        DOWN,
        RIGHT,
        LEFT;
    }


    // Constructor you call when doing new DPadButton()
    // Takes the input from the BobController in the form of a button, and the direction you want this button to represent
    public DPadButton( DoubleSupplier input, Direction dir ){
        in = input;
        // Converts Direction to number
        switch( dir ){
            case UP:
                dirNum = 0;
                break;
            case DOWN:
                dirNum = 180;
                break;
            case LEFT:
                dirNum = 270;
                break;
            case RIGHT:
                dirNum = 90;
                break;
            default:
                dirNum = -1;
                break;
        }

    }
    
    // Overrides the original get() function from the parent class (Button)
    @Override
    // If the value of the DPad = the value of the DPad you want, then give true
    public boolean get(){
        return in.getAsDouble() == dirNum;
    }
}