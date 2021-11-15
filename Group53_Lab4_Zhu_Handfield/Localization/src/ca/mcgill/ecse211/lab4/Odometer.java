/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.lab4;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

	private double x;
	private double y;
	private double theta;
	private double rad;

	private double width;
	private double Radius;
	private double diff = 0;
	private double dist = 0;

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RAD;

	private double[] position;

	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and
	 * variables once.It cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions {
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.rad = 0.0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		this.Radius = lab4.WHEEL_RAD;
		this.width = lab4.TRACK;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods
	 * provided from the OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		double X = 0;
		double Y = 0;
		double Theta = 0;
		double nowTachoL = 0;
		double nowTachoR = 0;
		double WR = WHEEL_RAD;
		double lastTachoL = 0;
		double lastTachoR = 0;
		double TR = TRACK;
		double dtheta;

		double distL, distR, deltaD, deltaT, dX, dY;

		while (true) {
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			// calculate new robot position based on tachometer counts
			nowTachoL = leftMotor.getTachoCount(); // get tacho counts for both left and right wheels
			nowTachoR = rightMotor.getTachoCount();
			distL = 3.14159 * WR * (nowTachoL - lastTachoL) / 180; // compute left wheel displacements
			distR = 3.14159 * WR * (nowTachoR - lastTachoR) / 180; // compute right wheel displacements
			lastTachoL = nowTachoL; // save left current tacho counts
			lastTachoR = nowTachoR; // save right current tacho counts
			deltaD = 0.5 * (distL + distR); // compute vehicle displacement using average value of summation of both
											// wheels
			deltaT = (distL - distR) / TR; // compute change in heading direction
			Theta += deltaT; // update heading
			dX = deltaD * Math.sin(Theta); // compute X component of displacement
			dY = deltaD * Math.cos(Theta); // compute Y component of displacement

			// update estimates of X and Y position
			dtheta = Math.toDegrees(deltaT);

			// update odometer values with new calculated values
			odo.update(dX, dY, dtheta);

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}
}
