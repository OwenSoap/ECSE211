package ca.mcgill.ecse211.lab3;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * this class implements a navigating system based on the odometers coordinates
 * (following the tracking of the wheels turning from lab 2) and adds in a US
 * poller with which an obstacle can be detected when the robot approaches an
 * obstacle too closely, a modified bang bang controller from lab1 method is
 * called after a safe amount of time avoiding the obstacle (as well as tracking
 * its movements) the robot then returns to its travelto method (from the new
 * point after avoiding) the travelto method calculates the distance and angle
 * that the robot must travel in to the next waypoint the travelto method
 * utilizes the turnto method which determines the minimal angle to turn towards
 * the next checkpoint
 * 
 * @author yizhu spencerhandfield
 */
public class Avoid extends Thread implements UltrasonicController {

	// declaring general variables
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private EV3MediumRegulatedMotor USMotor;
	private Object lock;
	private boolean navigating;
	private final int FORWARD_SPEED = 180;
	private final int ROTATE_SPEED = 150;
	private static int distance;

	// variables for bangbang
	private static final int bandCenter = 35; // Offset from the wall (cm)
	private static final int bandwidth = 3; // Width of dead band (cm)
	private static final int motorLow = 150; // Speed of slower rotating wheel (deg/sec)
	private static final int motorHigh = 230; // Speed of the faster rotating wheel (deg/seec)

	// variables for filter
	private final int FILTER_OUT = 20;
	private int filterControl = 0;
	private boolean obstacle;

	/**
	 * this instantiates all the motors and sensors the robot uses
	 * 
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param USMotor
	 */
	public Avoid(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3MediumRegulatedMotor USMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.USMotor = USMotor;
		this.lock = new Object();
	}

	/**
	 * this gives run gives the robot the list of checkpoints it must traverse
	 * through the map (the avoidance occurs within the route to each of such points
	 */
	public void run() {
		try {
			Thread.sleep(1500); // wait a second and a half before starting to move
		} catch (Exception e) {
		}
		synchronized (lock) { // lock to ensure only 1 is called at a time
			travelTo(0, 60);
			travelTo(30, 30);
			travelTo(60, 60);
			travelTo(60, 30);
			travelTo(30, 0);

		}
	}

	/**
	 * this is the travelto method which calculates how far the robot must travel
	 * based on its objective as well as it goal it then uses the turnto towards
	 * that goal and sets the wheels in motion for the distance required to move
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) { // makes robot travel to absolute location

		navigating = true; // setting navigating boolean active

		double dFromX, dFromY, theta;
		dFromX = x - odometer.getX(); // value of the distance of target x coordinate to
										// the current x coordinate
		dFromY = y - odometer.getY(); // value of the distance of target y coordinate to
										// the current y coordinate

		if (dFromY < 0 && dFromX < 0) {
			theta = Math.atan(dFromX / dFromY) - Math.PI;
		} else if (dFromY < 0 && dFromX >= 0) {
			theta = Math.atan(dFromX / dFromY) + Math.PI;
		} else {
			theta = Math.atan(dFromX / dFromY);
		}
		// get angle the robot has to turn

		turnTo(theta); // method to make robot turn to the required angle

		while (Math.abs(dFromX) > 0.5 || Math.abs(dFromY) > 0.5) { // the robot is more than 0.5 cm from the target
																	// coord.
			if (distance < 20) { // US detects an obstacle at a
									// distance of 20 cm or less
				obstacle = true;
				navigating = false;
				while (obstacle) {
					if (distance < 20) {
						// rotate motor for optimal bang bang usage
						USMotor.rotateTo(60);
						// rotate robot so sensor is still pointing towards obstacle
						leftMotor.rotate(convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, 45.0), true);
						rightMotor.rotate(-convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, 45.0), false);
						// call the bangbang method to avoid the obstacle
						bangbang();
						// beep upon completion of avoidance
						Sound.beepSequenceUp();
						// reset the sensor to in front of the robot
						USMotor.rotateTo(0);
						// return to traveling to the current waypoint objective
						travelTo(x, y);
					}
				}
			} // end of avoiding obstacle

			// constantly update distance to travel
			dFromX = x - odometer.getX();
			dFromY = y - odometer.getY();
			// if the robot drifts too far off its desired angle
			if (Math.abs(odometer.getTheta() - theta) >= 0.5) {
				turnTo(theta); // turn back to the desired theta
			}

			// move robot forwards to the waypoint based on the distances from
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();

			// constantly update the distance from for the condition to exit the travelling
			// loop
			dFromX = x - odometer.getX();
			dFromY = y - odometer.getY();

		}
		// checkpoint reached, completely stop the robot before entering the next
		// travelto
		navigating = false;
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * a modified version of the bang bang from lab1, variables created locally in
	 * this class put on a tested timer for effective avoidance
	 */
	private void bangbang() {

		if (distance > 2000) // capping distance to prevent too large of an integer turning into negative
			// number, tested to prevent too large int turning into negative float
			distance = 230; // resetting the distance to create manageable variables

		int error = distance - bandCenter; // difference between distance and center
		int miner = bandCenter - bandwidth; // minimum acceptable distance
		int maxer = bandCenter + bandwidth; // maximum acceptable distance

		long start = System.currentTimeMillis();
		long end = start + 8 * 1000; // 8 seconds
		while (System.currentTimeMillis() < end) {
			// all turning speeds per case are constant regardless of distance within each
			// from wall as per specification
			// System.out.println("timer" + distance);
			if (Math.abs(error) <= bandwidth) { // robot is within the desired distance from the wall
				leftMotor.setSpeed(motorHigh); // Start robot moving forward
				rightMotor.setSpeed(motorHigh);
				leftMotor.forward();
				rightMotor.forward();
			} else if (distance < 10) { // the robot is dangerously too close to the wall
				rightMotor.setSpeed(motorHigh * 3); // back away from the wall immediately
				leftMotor.setSpeed(motorHigh * 2); // wheels at different speed
				leftMotor.backward(); // different speed slightly turns the robot, so as to better adjust trajectory
				rightMotor.backward();
			} else if (distance - miner < 0) { // the robot is too close to the wall
				leftMotor.setSpeed(motorHigh * 3); // simply turn back right, away from the wall
				rightMotor.setSpeed(motorLow); // speed of right higher to increase avoidance of collision
				leftMotor.forward();
				rightMotor.forward();
			} else if (distance - maxer > 0) { // the robot is too far from the wall
				leftMotor.setSpeed(motorLow); // gently turn back towards wall
				rightMotor.setSpeed(motorHigh); // speeds lower so as to prevent over correction
				leftMotor.forward();
				rightMotor.forward();
			}
		}
	}

	/**
	 * this function computes the minimal angle of that the robot must turn towards
	 * the waypoint takes current distance and current angle
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) { // makes robot turn to absolute heading
										// theta
		navigating = true;
		double delTheta = theta - odometer.getTheta();
		double delThetaDeg = Math.toDegrees(delTheta);
		double minAngle;
		// arithmatic for min angle
		if (delThetaDeg < -180) {
			minAngle = delThetaDeg + 360;
		} else if (delThetaDeg > 180) {
			minAngle = delThetaDeg - 360;
		} else {
			minAngle = delThetaDeg;
		}
		// minimal angle code
		// rotates the robot towards the minimal angle
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, minAngle), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, minAngle), false);
		// navigating false while turning
		navigating = false;
	}

	/**
	 * boolean for when navigating is active
	 * 
	 * @return
	 */
	public boolean navigating() { // true if another thread has called
									// travelTo() or turnTo() and the method has
									// yet to return, else false
		return this.navigating;
	}

	/**
	 * this method converts the angle into the value for wheel rotation
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
		// convert angles to appropriate units
	}

	/**
	 * method converts the distance into an angle to turn at
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
		// convert distance to appropriate units
	}

	/**
	 * read the US data
	 */
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		if (distance >= 60 && filterControl < FILTER_OUT) {
			filterControl++;
		} else if (distance >= 70) {
			obstacle = false;
			this.distance = distance;
		} else {
			filterControl = 0;
			this.distance = distance;
		}
	}

	/**
	 * set US data
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
