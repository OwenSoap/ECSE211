package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this class extends thread the navigation class is to calculate the
 * differences between target coordinates and current coordinates to move the
 * robot from current to the specific one
 * 
 * @author yizhu spencerhandfield
 *
 */
public class Navigation extends Thread {
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Object lock;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	public static final double TILE_SIZE = 30.48;
	double track = Lab3.TRACK;
	double radius = Lab3.WHEEL_RADIUS;

	/**
	 * this method is the default constructor for Navigation it takes in all the
	 * relevant motors and odometer method from Lab3 package to navigate the robot
	 * to target coordinates
	 * 
	 * @param odometer
	 * @param leftMotor
	 * @param mrightMotor
	 */
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.lock = new Object();
	}

	/**
	 * Here is where the navigation code should be run. Using two arrays to store
	 * relative coordinates of target points and using odometry data to get the
	 * distance
	 */
	public void run() {

		int[] goToX = { 0, 1, 1, 2, 2 }; // points to be visited X
		int[] goToY = { 1, 2, 0, 1, 2 }; // points to be visited Y
		int visitPoint = 5; // total number of points to visit
		int curPoint = 0; // initialize current number of point
		// initialize the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(800);
		}
		// wait 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {

		}
		// if current points are less than total target points and the robot is not
		// moving
		// move to next target point
		while (curPoint < visitPoint) {
			if (this.isNavigating() == false) {
				travelTo(goToX[curPoint] * TILE_SIZE, goToY[curPoint] * TILE_SIZE);
				curPoint++;
			}
		}
	}

	/**
	 * this is the method to travel to target coordinates
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) { // makes robot travel to absolute field location
		// initialize parameters
		double dFromX;
		double dFromY;
		double distance;
		double theta;
		double curx = odometer.getX(); // get current x position
		double cury = odometer.getY(); // get current y position
		int forwardOffset = 3; // forward moving offset for right motor
		dFromX = x - curx;
		dFromY = y - cury;

		// calculate the angle theta to turn
		if (dFromY < 0 && dFromX < 0) {
			theta = ((Math.atan(dFromX / dFromY)) - Math.PI);
		} else if (dFromY < 0 && dFromX > 0) {
			theta = ((Math.atan(dFromX / dFromY)) + Math.PI);
		} else {
			theta = Math.atan(dFromX / dFromY);
		}

		// heading to the target point first
		turnTo(theta);

		// calculate the distance to the target point
		distance = calculateDistance(dFromX, dFromY);
		// start the motors
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED + forwardOffset);
		leftMotor.rotate(convertDistance(radius, distance), true);
		rightMotor.rotate(convertDistance(radius, distance), false);
	}

	/**
	 * this is the method to turn the robot to target point if the angle is greater
	 * than 180 degrees, it will automatically change to another minimal angle to
	 * turn
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {
		double delTheta = theta - odometer.getTheta(); // get angle difference
		double delThetaMin;
		int rotateOffset = 5; // moving offset for rotation
		// make sure turn minimum angle
		if (delTheta < -Math.PI) {
			delThetaMin = Math.toDegrees(delTheta + 2 * Math.PI);
		} else if (delTheta > Math.PI) {
			delThetaMin = Math.toDegrees(delTheta - 2 * Math.PI);
		} else {
			delThetaMin = Math.toDegrees(delTheta);
		}

		// start motor to turn
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED + rotateOffset);
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, delThetaMin), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RADIUS, Lab3.TRACK, delThetaMin), false);
	}

	/**
	 * this is the method to check whether the robot is moving
	 * 
	 * @return isNavigating
	 */
	public boolean isNavigating() {
		boolean isNavigating = false;
		if (leftMotor.isMoving() || rightMotor.isMoving()) {
			isNavigating = true;
		}
		return isNavigating;
	}

	/**
	 * using mathematic method to calculate distance to target point
	 * 
	 * @param xdiff
	 * @param ydiff
	 * @return distance
	 */
	public static double calculateDistance(double xdiff, double ydiff) {
		return Math.sqrt(Math.pow(xdiff, 2) + Math.pow(ydiff, 2));
	}

	/**
	 * this method convert distance for wheels to rotate
	 * 
	 * @param radius
	 * @param distance
	 * @return numbers of wheels to rotate
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * this is the method to calculate the angle for robot to turn
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return numbers of wheels to rotate
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}