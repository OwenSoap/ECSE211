package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.lab4;
import lejos.hardware.*;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends OdometerData implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private EV3ColorSensor colorSensor;
	private SampleProvider colorIDSensor;
	private double firstBack = 16;
	private double secondBack = 16.5;
	public double data;
	private static int speed = lab4.FORWARD_SPEED;
	private static double radius = lab4.WHEEL_RAD;
	private static double track = lab4.TRACK;
	public static float lastclr;
	private boolean firstCorrect = true;
	private boolean secondCorrect;
	private boolean blackDetected = false;
	private int colorGot;
	int counter;

	/**
	 * this is the shell class for the lightlocalizer and sets all its components
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param odometer
	 * @param colorSensor
	 * @throws Exception
	 */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			EV3ColorSensor colorSensor) throws Exception {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.colorSensor = colorSensor;
		this.colorIDSensor = colorSensor.getColorIDMode();
		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);

	}

	/**
	 * this run method begins the light localizing, sets the offset of the light
	 * sensor relative to the middle of the robot and contains the break condition
	 * for finishing the process
	 */
	public void run() {

		lab4.leftMotor.setSpeed(speed);
		lab4.rightMotor.setSpeed(speed);
		while (true) {
			
			colorGot = colorSensor.getColorID(); // store colorID into colorGot;
			locaLize(colorGot);

			if (secondCorrect == false) { // exit the loop
				break;
			}
		}
	}

	/**
	 * the localize class takes in the color received from the light sensor, upon
	 * detecting a line, a different value is passed in the robot will detect the
	 * line representing the x axis, adjust to light sensor displacement (measured
	 * to be 16) it will then turn, detect the line representing the y axis and
	 * adjust for light sensor displacement
	 * 
	 * @param x
	 */
	public void locaLize(int x) {

		// First line not detected yet
		if (firstCorrect == true && lineDetected(x) == false) {

			// Advance indefinitely
			lab4.leftMotor.rotate(convertDistance(lab4.WHEEL_RAD, 50), true);
			lab4.rightMotor.rotate(convertDistance(lab4.WHEEL_RAD, 50), true);
			secondCorrect = true;

			// First line detected by light sensor via integer input changing to known value
			// of black line
		} else if (firstCorrect == true && lineDetected(x) == true) {

			leftMotor.stop(true);
			rightMotor.stop(false);
			// Reduce potential for error by stopping and waiting for 0.5 seconds
			try {
				Thread.sleep(500);
			} catch (Exception e) {

			}

			move(false, firstBack); // Adjust position to compensate for light sensor offset (measured to be 16)
			odometer.setY(0); // Odometer correction, robot should now be along the x axis, meaning its Y is
								// zero
			turn(true, 90); // Turn along x axis to begin localizing to the Y axis to correct the x value
			counter = 1; // Count for lines detected
			secondCorrect = true;
			firstCorrect = false; // Adjust variables for next step

		} else if (counter == 1 && lineDetected(x) == false) {

			lab4.leftMotor.rotate(convertDistance(lab4.WHEEL_RAD, 50), true);
			lab4.rightMotor.rotate(convertDistance(lab4.WHEEL_RAD, 50), true);
			secondCorrect = true; // Ready for second correction scan

		} else if (counter == 1 && lineDetected(x) == true) { // Detected second black line
			// Stop and pause
			leftMotor.stop(true);
			rightMotor.stop(false);
			try {
				Thread.sleep(500);
			} catch (Exception e) {

			}
			move(false, secondBack); // Adjust for light sensor offset and odometer inaccuracy measured at 16.5
			odometer.setX(0); // Correct the x reading as it should now be at the origin
			turn(false, 90); // Turn 90 degrees to face forward along y axis from the origin
			counter = 2; // Count for lines detected
			odometer.setXYT(0, 0, 0); // Correct all values to origin value
			secondCorrect = false; // Create condition for program exit
		}
	}

	public boolean lineDetected(int x) {
		while (true) {

			// Determines based on color ID if line was detected
			if (x > 12) {
				blackDetected = true; // If so return true
				Sound.beepSequenceUp();

			} else {
				blackDetected = false; // No line detected returns false
			}
			return blackDetected;
		}

	}

	/**
	 * a method taken from square driver that rotates the robot in place based on
	 * angle provided
	 * 
	 * @param direction
	 * @param angle
	 */
	public void turn(boolean direction, double angle) {

		if (direction == true) { // If direction = true turn right
			lab4.leftMotor.rotate(convertAngle(radius, track, angle), true);
			lab4.rightMotor.rotate(-convertAngle(radius, track, angle), false);
		} else { // Else turn right
			lab4.leftMotor.rotate(-convertAngle(radius, track, angle), true);
			lab4.rightMotor.rotate(convertAngle(radius, track, angle), false);
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * a method from previous lab to move robot based on input distance
	 * 
	 * @param direction
	 * @param distance
	 */
	public void move(boolean direction, double distance) {
		lab4.leftMotor.setSpeed(speed);
		lab4.rightMotor.setSpeed(speed);

		if (direction == true) { // Direction true then move forward
			lab4.leftMotor.rotate(convertDistance(lab4.WHEEL_RAD, distance), true);
			lab4.rightMotor.rotate(convertDistance(lab4.WHEEL_RAD, distance), false);
		}

		else { // Else move backward
			lab4.leftMotor.rotate(-convertDistance(lab4.WHEEL_RAD, distance), true);
			lab4.rightMotor.rotate(-convertDistance(lab4.WHEEL_RAD, distance), false);
		}
	}

	/**
	 * method taken from previous lab to convert angle between rads and degrees for
	 * various components
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * method taken from previous lab to convert distance from rads to cm for
	 * various components
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}
