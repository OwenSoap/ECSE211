package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {

	// initialize robot position variable
	private double x, y, theta, thetaDisp;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final int degToRad = 1745; // ratio of pi/180 multiplied by 100000 for accuracy

	private int tachoR;
	private int tachoL;
	private int lastTachoR = 0;
	private int lastTachoL = 0;

	// odometer update period in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for synchronization
	private Object lock;

	/**
	 * this is the class for odometer to initialize motors status
	 * @param leftMotor
	 * @param rightMotor
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		lock = new Object();

		// clear tacho counts and then initialize the tacho count to its current state.
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		lastTachoL = leftMotor.getTachoCount();
		lastTachoR = rightMotor.getTachoCount();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();

			synchronized (lock) {

				// read current left and right wheels tacho counts and save them
				tachoL = leftMotor.getTachoCount();
				tachoR = rightMotor.getTachoCount();

				// distance traveled by right wheel
				double distRight = Lab3.WHEEL_RADIUS * ((tachoR - lastTachoR) * degToRad) / 100000;

				// distance traveled by left wheel
				double distLeft = Lab3.WHEEL_RADIUS * ((tachoL - lastTachoL) * degToRad) / 100000;

				// update the tacho count
				lastTachoL = tachoL;
				lastTachoR = tachoR;

				// calculate the distance traveled the robot found by averaging distance
				// traveled by each wheel
				double deltaD = (distRight + distLeft) / 2;

				// calculate the angle to make
				double deltaT = (distLeft - distRight) / Lab3.TRACK;

				// update theta by adding angle rotated
				theta += deltaT;

				// update x and y positions by adding deltaD
				x += deltaD * Math.sin(theta);
				y += deltaD * Math.cos(theta);
				thetaDisp += Math.toDegrees(deltaT);
				if (thetaDisp > 360) {
					thetaDisp = thetaDisp - 360;
				} else if (thetaDisp < 0) {
					thetaDisp = thetaDisp + 360;
				}

			}

			// this method ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// nothing to be done here
				}
			}
		}
	}

	/**
	 * this is a method to get the x, y and theta position of odometery
	 * 
	 * @param position
	 * @param update
	 */
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	/**
	 * getX method
	 * 
	 * @return x
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/**
	 * getY method
	 * 
	 * @return y
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	/**
	 * getTheta method
	 * 
	 * @return theta
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	/**
	 * convert theta to degree for display method
	 * 
	 * @return thetaDisp
	 */
	public double getThetaDisp() {
		double result;

		synchronized (lock) {
			result = thetaDisp;
		}

		return result;
	}

	/**
	 * 
	 * @param position
	 * @param update
	 */
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * setX method
	 * 
	 * @param x
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	/**
	 * setY method
	 * 
	 * @param y
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	/**
	 * setTheta method
	 * 
	 * @param theta
	 */
	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * setThetaDisp method
	 * 
	 * @param thetaDisp
	 */
	public void setThetaDisp(double thetaDisp) {
		synchronized (lock) {
			this.thetaDisp = thetaDisp;
		}
	}
}
