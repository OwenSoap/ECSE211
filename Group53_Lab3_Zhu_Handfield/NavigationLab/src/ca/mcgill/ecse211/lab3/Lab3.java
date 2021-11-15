package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.Odometer;

import ca.mcgill.ecse211.lab3.OdometryDisplay;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

/**
 * this is the main class for lab3
 * 
 * @author yizhu spencerhadfield
 *
 */
public class Lab3 {

	// Initialize ports
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3MediumRegulatedMotor USMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

	// Parameters: adjust these for desired performance
	public static final double WHEEL_RADIUS = 2.2;
	public static final double TRACK = 11.6;

	public static void main(String[] args) {
		int buttonChoice;

		@SuppressWarnings("resource")
		// Initialize ultrasoinic sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];
		UltrasonicPoller usPoller = null;

		// Initialize all instances
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Avoid avoid = new Avoid(odometer, leftMotor, rightMotor, USMotor);
		Navigation navigator = new Navigation(odometer, leftMotor, rightMotor);
		usPoller = new UltrasonicPoller(usDistance, usData, avoid);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t, avoid);

		do {
			// clear the display
			t.clear();

			// Ask the user whether the motors should Avoid Block or Go to specific
			// locations
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" Avoid | Drive  ", 0, 2);
			t.drawString(" Block | to loc   ", 0, 3);
			t.drawString("       | ations", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			t.clear();// Clear the display
			Printer printer = new Printer(buttonChoice, avoid);// Invoke printer class for distance display
			// Start all threads
			odometer.start();
			odometryDisplay.start();
			printer.start();
			usPoller.start();
			avoid.start();

		} else {
			// Start all threads
			odometer.start();
			odometryDisplay.start();
			usPoller.start();
			navigator.start();
		}
		// Press any buttom to exit the program
		Button.waitForAnyPress();
		System.exit(0);
	}

}
