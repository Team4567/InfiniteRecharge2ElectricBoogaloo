/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot;

// Ah yes, PID.
public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
	
	/**
	 * https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html#:~:text=PID%20Control%20lies%20at%20the,sensors%2C%20light%20sensors%2C%20etc.
	 * @param _kP See link.
	 * @param _kI See link.
	 * @param _kD See link.
	 * @param _kF See link.
	 * @param _kIzone The zone around the target at which I will actually be in use. If error is too big at the beginning, I could become way too powerful too fast.
	 * @param _kPeakOutput will most likely be 1, represntes a max between 0 (None) to 1 (Full)
	 */
	public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}
}