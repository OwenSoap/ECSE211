package ca.mcgill.ecse211.lab4;

import java.util.*;

import ca.mcgill.ecse211.lab4.lab4;
import lejos.hardware.*;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Move;

public class UltronsanicLocalizer extends Thread implements UltrasonicController {

	private int distance;
	private double dangerDis = 40;
	private double risTurn = 215.4;
	private double falTurn = 40.7;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private final int FILTER_OUT = 20;
	private static int speed = lab4.FORWARD_SPEED;
	private static int ROTATE_SPEED = lab4.ROTATE_SPEED;
	private static double radius = lab4.WHEEL_RAD;
	private static double track = lab4.TRACK;
	private SampleProvider us;
	private float[] usData;
	public boolean edge;
	int counter;
	double angleTurned;
	boolean firstDetect = true;
	boolean secondDetect = true;

	/**
	 * This defines the variables and components of the USLocalizer class
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param odometer
	 * @param us
	 * @param usData
	 */
	public UltronsanicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			SampleProvider us, float[] usData) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.us = us;
		this.usData = usData;
		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
	}

	/**
	 * This begins the USLocalizer, it first acquires all the data from the hardware
	 * components then based on the button pressed, enters the appropriate rising or
	 * falling edge method
	 */
	public void run() {

		while (true) {
			// Acquire data
			us.fetchSample(usData, 0);
			distance = (int) (usData[0] * 100.0); // Get data
			LCD.drawString(Integer.toString(distance), 0, 4);
			// Based on user input selects appropriate method
			if (lab4.edge == true) { // Rising edge mode
				risingEdge(distance);
			} else { // Falling edge mode
				fallingEdge(distance);

			}
			// If finished then set theta value to be 0
			if (secondDetect == false) {
				odometer.setTheta(0);
				break;

			}
		}

	}

	/**
	 * This is the rising edge method which takes in the distance detected by the US
	 * it begins facing the wall and looks for the rising edge as defined by the
	 * wall being 40cm away it calculates the theta between those two edges and
	 * localizes the robots angle after detecting the first edge, there is a hard
	 * coded filter to turn back towards the wall
	 * 
	 * @param distance
	 */
	public void risingEdge(int distance) {

		lab4.leftMotor.setSpeed(ROTATE_SPEED);
		lab4.rightMotor.setSpeed(ROTATE_SPEED);

		// Number of times see the wall
		if (firstDetect) {
			lab4.leftMotor.rotate(-convertAngle(radius, track, 360), true); // Rotate until wall is "left"
			lab4.rightMotor.rotate(convertAngle(radius, track, 360), true);

			if (distance > dangerDis) { // Rising edge detected
				counter = 1; // Count detection
				firstDetect = false; // Exit the first detection turn
			}
			secondDetect = true;

		}
		
		// this loop serves as a sort of filter
		// the first edge was detected and it will begin to turn back towards the next
		// to compensate with the US being non-ideal, to avoid a simple for loop with
		// distance as argument being falsely triggered
		// quasi-immediately as the sensor returns from the edge, a preset amount of
		// distance will be turned back
		// thus ensuring it is facing a wall and will only count the next edge when it
		// is actually there and not
		// in the potentially falsefield readings during the detection of the first edge
		else if (distance > dangerDis && counter == 1) {

			leftMotor.stop(true); // Stop rotating
			rightMotor.stop(false);
			odometer.setTheta(0); // Reset theta to 0 to begin counting the angle between the edges
			Sound.beepSequenceUp();
			lab4.leftMotor.rotate(convertAngle(radius, track, 60), true); // Turn back towards the second edge a
																			// predetermined amount to avoid error
			lab4.rightMotor.rotate(-convertAngle(radius, track, 60), false);
			Sound.beep();
			counter = 2; // Set counter to enter next loop
			secondDetect = true; // Still awaiting second edge

			// After detecting first rising edge, and safely turning back towards the wall
			// to read distance again below threshold
			// Poll for next edge
		} else if (distance < dangerDis && counter == 2) {

			lab4.leftMotor.rotate(convertAngle(radius, track, 360), true); // Turn indefinitely until next edge
			lab4.rightMotor.rotate(-convertAngle(radius, track, 360), true);
			secondDetect = true; // Second edge still not detected

			// Second edge detected
		} else if (distance > dangerDis && counter == 2) {

			leftMotor.stop(true); // Stop
			rightMotor.stop(false);
			angleTurned = odometer.getXYT()[2]; // Determine theta between two edges
			Sound.beepSequenceUp();
			turn(false, 0.5 * angleTurned); // Force robot to face 45 degree
			turn(false, risTurn); // Facing forward along , along 0 axis
			leftMotor.stop(true);
			rightMotor.stop(false);
			counter = 3;
			secondDetect = false;

		}

	}

	/**
	 * the falling edge method begins facing away from the wall it turns towards the
	 * corner until the first edge is detected (wall is less than 40cm away) then
	 * utilizes a similar filter and rising edge to safely turn away from wall
	 * before polling for next edge detects the second edge and adjusts robot's
	 * theta along the axis
	 * 
	 * @param distance
	 */
	public void fallingEdge(int distance) {

		lab4.leftMotor.setSpeed(ROTATE_SPEED);
		lab4.rightMotor.setSpeed(ROTATE_SPEED);

		// Begin turning towards the wall for falling edge detection
		if (firstDetect) {
			lab4.leftMotor.rotate(-convertAngle(radius, track, 360), true); // Turns left until first edge found
			lab4.rightMotor.rotate(convertAngle(radius, track, 360), true);

			if (distance < dangerDis) { // First edge located
				counter = 1; // Set variables to enter next loop
				firstDetect = false;
			}
			secondDetect = true;

		}

		// Like previous filter, turns back a known distance to prevent false US
		// readings near the edges
		else if (distance < dangerDis && counter == 1) {

			leftMotor.stop(true);
			rightMotor.stop(false);
			odometer.setTheta(0); // Set theta 0 to begin measuring theta between edges
			Sound.beepSequence();
			lab4.leftMotor.rotate(convertAngle(radius, track, 70), true); // Turns right, away from the wall towards
																			// next edge
			lab4.rightMotor.rotate(-convertAngle(radius, track, 70), false);
			Sound.beep();
			counter = 2; // Define variables for next loop
			secondDetect = true;

			// Continue rotating while safely facing away from wall until next edge found
		} else if (distance > dangerDis && counter == 2) {

			lab4.leftMotor.rotate(convertAngle(radius, track, 360), true);
			lab4.rightMotor.rotate(-convertAngle(radius, track, 360), true);
			secondDetect = true;
			// Second falling edge located
		} else if (distance < dangerDis && counter == 2) {

			leftMotor.stop(true);
			rightMotor.stop(false);
			angleTurned = odometer.getXYT()[2]; // Acquire theta between two edges
			Sound.beepSequence();
			turn(false, 0.5 * angleTurned); // Adjust robots orientation to be 0 degrees, along the axis based on angle
										// between edges
			turn(false, falTurn);
			leftMotor.stop(true);
			rightMotor.stop(false);
			counter = 3;
			secondDetect = false;

		}

	}

	/**
	 * a method taken from square driver that rotates the robot in place based on
	 * angle provided
	 * 
	 * @param direction
	 * @param angle
	 */
	public static void turn(boolean direction, double angle) {

		lab4.leftMotor.setSpeed(ROTATE_SPEED);
		lab4.rightMotor.setSpeed(ROTATE_SPEED);

		if (direction == true) { // Turn right if direction is true
			lab4.leftMotor.rotate(convertAngle(radius, track, angle), true);
			lab4.rightMotor.rotate(-convertAngle(radius, track, angle), false);
		} else { // Else turn left
			lab4.leftMotor.rotate(-convertAngle(radius, track, angle), true);
			lab4.rightMotor.rotate(convertAngle(radius, track, angle), false);
		}
	}

	public static void move(boolean direction, double distance) {

		lab4.leftMotor.setSpeed(speed);
		lab4.rightMotor.setSpeed(speed);

		if (direction == true) { // Move forward if direction is true
			lab4.leftMotor.rotate(convertDistance(lab4.WHEEL_RAD, distance), true);
			lab4.rightMotor.rotate(convertDistance(lab4.WHEEL_RAD, distance), false);
		}

		else { // Else move backward
			lab4.leftMotor.rotate(-convertDistance(lab4.WHEEL_RAD, distance), true);
			lab4.rightMotor.rotate(-convertDistance(lab4.WHEEL_RAD, distance), false);
		}
	}

	/**
	 * Method taken from previous lab to convert angle between rads and degrees for
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
	 * a method from previous lab to move robot based on input distance
	 * 
	 * @param direction
	 * @param distance
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Filter and process inspired from previous labs
	 */
	@Override
	public void processUSData(int distance) {

		int filterControl = 0;

		if (distance >= 255 && filterControl < FILTER_OUT) {
			// Problematic, potentially false reading
			filterControl++;// Increment wait to actually determine if previous reading is true
		} else if (distance >= 255) {
			// The large value has been repeated long enough to be determined as actual
			// value
			this.distance = distance;
		} else {
			// Distance went back below the problematic reading signaling a false data entry
			filterControl = 0; // Reset the filter counter for next
			this.distance = distance; // Maintain current accurate data
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
