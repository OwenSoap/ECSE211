/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private double lightSensorDis = 4;
	private double squareSideLen = 30.48;
	private static final Port portColor = LocalEV3.get().getPort("S1");
	private static SensorModes myColor = new EV3ColorSensor(portColor);
	private static float color = 0;
	private int count = 0;

	// static SampleProvider myColorSample = lightSensor.getRedMode();
	// float [] sample = new float[myColorSample.sampleSize()];

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

	}

	/**
	 * Here is where the odometer correction code should be run. Using the light
	 * sensor and the tile lines to better measure the distance and ajust
	 * accordingly for the robot we can count the amount of lines crossed and with
	 * known tile size, supply proper coordinates origin is defined at the corner of
	 * the square that the robot begins in the middle of
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {

		// These are the direction vectors. 0 = not moving, 1 = Towards increasing
		// values, -1 = Towards decreasing values.

		long correctionStart, correctionEnd;
		SampleProvider myColorSample = myColor.getMode("Red");
		float[] sampleColor = new float[myColor.sampleSize()];
		myColorSample.fetchSample(sampleColor, 0);
		color = sampleColor[0] * 1000;

		while (true) {
			correctionStart = System.currentTimeMillis();

			myColorSample.fetchSample(sampleColor, 0);
			color = sampleColor[0] * 1000;
			// TODO Place correction implementation here

			// only corrects based on crossing one of the black tile lines
			if (color < 450) {
				Sound.beep(); // beep to signal line crossed
				double theta = odometer.getXYT()[2]; // acquire current theta
				theta = theta * Math.PI / 180; // convert
				count++; // increase LINE COUNTING
				double sensorOffset = 0; // define a correction to hardware placement

				// all if statements check for how many tiles crossed
				// depending on amount crossed, with known tile size, corrects the odometer
				// extra && correction to ensure proper orientation regardless of count
				if (count == 1 && theta >= -(Math.PI / 4) && theta < Math.PI / 4) {
					sensorOffset = Math.cos(theta) * lightSensorDis;
					odometer.setY(0 - sensorOffset); // set to 0 based on defined origin (Y-axis orientation) and the
														// physical placement of sensor vs center of robot
					odometer.setTheta(0);// is going straight

					// repeat process for each successive line cross, adjusting the odometer
					// properly
				} else if (count == 2 && theta >= -(Math.PI / 4) && theta < Math.PI / 4) {
					sensorOffset = Math.cos(theta) * lightSensorDis;
					odometer.setY(squareSideLen - sensorOffset);// know we have gone 1 full tile from origin, adjust
					odometer.setTheta(0);// reset direction straight

					// continuously adjust for each crossed line
				} else if (count == 3 && theta >= -(Math.PI / 4) && theta < Math.PI / 4) {
					sensorOffset = Math.cos(theta) * lightSensorDis;
					odometer.setY(2 * squareSideLen - sensorOffset); // have travelled 2 full tiles from origin
					odometer.setTheta(0); // ensure right orientation

					// after 4, turn has occured, can set X coordinate to 0 as it crossed defined X
					// axis
				} else if (count == 4 && theta >= Math.PI / 4 && theta < 3 * Math.PI / 4) {
					sensorOffset = Math.sin(theta) * lightSensorDis;
					odometer.setX(0 - sensorOffset); // set origin, adjust for physical hardware placement
					odometer.setTheta(90); // perpendicular to starting direction, parallel to X-axis now

				} else if (count == 5 && theta >= Math.PI / 4 && theta < 3 * Math.PI / 4) {
					sensorOffset = Math.sin(theta) * lightSensorDis;
					odometer.setX(squareSideLen - sensorOffset); // crossed 1 tile in X-direction
					odometer.setTheta(90); // proper direction

				} else if (count == 6 && theta >= Math.PI / 4 && theta < 3 * Math.PI / 4) {
					sensorOffset = Math.sin(theta) * lightSensorDis;
					odometer.setX(2 * squareSideLen - sensorOffset);// crossed 2 tile in X-direction
					odometer.setTheta(90);

				} else if (count == 7 && theta >= 3 * Math.PI / 4 && theta < 3 * Math.PI / 2) {
					sensorOffset = Math.cos(theta) * lightSensorDis;
					odometer.setY(2 * squareSideLen - sensorOffset);// based on origin definition, crossing this line is
																	// known max 2 tile away distance
					odometer.setTheta(180); // turn has occured

				} else if (count == 8 && theta >= 3 * Math.PI / 4 && theta < 3 * Math.PI / 2) {
					sensorOffset = Math.cos(theta) * lightSensorDis;
					odometer.setY(squareSideLen - sensorOffset);// 1 tile closer to origin = 1 tile away still
					odometer.setTheta(180);// travelling in negative Y-direction

				} else if (count == 9 && theta >= 3 * Math.PI / 4 && theta < 3 * Math.PI / 2) {
					sensorOffset = Math.cos(theta) * lightSensorDis;
					odometer.setY(0 - sensorOffset);// returned to defined Y-axis
					odometer.setTheta(180);
					// will continue measuring the small distance travelled to return to point S
					// this value is measured from 0 after knowing it has properly crossed the line

				} else if (count == 10 && theta >= 5 * Math.PI / 4 && theta < 7 * Math.PI / 4) {
					sensorOffset = Math.sin(theta) * lightSensorDis;
					odometer.setX(2 * squareSideLen - sensorOffset);// crossing this line is furthest precise known X
																	// value
					odometer.setTheta(270);// returning to starting point after 3 turns

				} else if (count == 11 && theta >= 5 * Math.PI / 4 && theta < 7 * Math.PI / 4) {
					sensorOffset = Math.sin(theta) * lightSensorDis;
					odometer.setX(squareSideLen - sensorOffset);// 1 tile closer to origin
					odometer.setTheta(270);

				} else if (count == 12 && theta >= 5 * Math.PI / 4 && theta < 7 * Math.PI / 4) {
					sensorOffset = Math.sin(theta) * lightSensorDis;
					odometer.setX(0 - sensorOffset);// crossed defined X-axis
					odometer.setTheta(270); // returns to S while continuing to calculate the X offset of S
				}

			}
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
