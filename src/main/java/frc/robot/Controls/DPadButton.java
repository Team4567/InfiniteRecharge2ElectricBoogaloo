package frc.robot.Controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

public class DPadButton extends Button{
    DoubleSupplier in;
    double dirNum;
    public enum Direction{
        UP,
        DOWN,
        RIGHT,
        LEFT;

       
    }

    public DPadButton( DoubleSupplier input, Direction dir ){
        in = input;
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
    
    @Override
    public boolean get(){
        return in.getAsDouble() == dirNum;
    }
}