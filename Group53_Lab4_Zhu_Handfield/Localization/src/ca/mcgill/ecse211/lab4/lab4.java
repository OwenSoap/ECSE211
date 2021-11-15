package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.SensorMode;

/**
 * following the logic set forth by previous labs, this is a central class that
 * instantiates all necessary components for all desired methods and objectives
 * and then based on user input determines which class and operation to call as
 * well as calling the order of the localiziation for the entirety of completing
 * the lab objectives
 * 
 * @author shand yi_zhu
 *
 */
public class lab4 extends Thread {
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); // define
																													// ports
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3ColorSensor colorPort = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.09;
	public static final double TRACK = 10.1;
	public static final int FORWARD_SPEED = 120;
	public static final int ROTATE_SPEED = 50;
	public static final double bandcenter = 10;
	public static boolean edge;

	@SuppressWarnings("deprecation")
	public static void main(String[] args) throws Exception {
		int choice;

		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];

		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd);
		UltronsanicLocalizer ultrosanicLocalizer = new UltronsanicLocalizer(leftMotor, rightMotor, odometer, usDistance,
				usData);
		LightLocalizer lightLocalizer = new LightLocalizer(leftMotor, rightMotor, odometer, colorPort);

		do {
			// Display default menu
			lcd.clear();
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Falling| Rising ", 0, 2);
			lcd.drawString(" Edge  |  Edge	", 0, 3);
			lcd.drawString("       |     	", 0, 4);
			choice = Button.waitForAnyPress();

		} while (choice != Button.ID_LEFT && choice != Button.ID_RIGHT);

		// Initialize all threads
		Thread odoThread = new Thread(odometer);
		Thread odoDisplayThread = new Thread(odometryDisplay);
		Thread UltrasonicLocalizerThread = new Thread(ultrosanicLocalizer);
		UltrasonicPoller usPoller = null;
		Thread LightLocalizer = new Thread(lightLocalizer);

		// Based on type of method called, set variables so US localizing class may
		// enter proper method
		if (choice == Button.ID_LEFT) {
			lcd.clear();
			edge = false;
			usPoller = new UltrasonicPoller(usDistance, usData, ultrosanicLocalizer);
			odoThread.start();
			odoDisplayThread.start();
			ultrosanicLocalizer.start(); // Begin selected US localizer
			// After US localizier, await further press to begin the light localizer
			while (Button.waitForAnyPress() != Button.ID_DOWN)
				; // Await further button press
			ultrosanicLocalizer.join(); // End US localizing
			lightLocalizer.start(); // Begin light localizing

		} else {
			lcd.clear();

			edge = true;
			usPoller = new UltrasonicPoller(usDistance, usData, ultrosanicLocalizer);

			odoThread.start();
			odoDisplayThread.start();
			ultrosanicLocalizer.start();

			while (Button.waitForAnyPress() != Button.ID_DOWN)
				; // Await further button press
			ultrosanicLocalizer.join(); // End US localizing
			System.out.println(ultrosanicLocalizer.isAlive());
			lightLocalizer.start(); // Begin light localizing

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}
