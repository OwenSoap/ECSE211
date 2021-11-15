package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Display;
import ca.mcgill.ecse211.lab5.Test;
import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class initiates and runs the desired programs
 * 
 * @author Maxime Cardinal
 */
public class Lab5 {
	
	//initiate all EV3 motors
	static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	//initiate important constants
	private static final TextLCD textlcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.09;
	public static final double TRACK = 11.40;
	public static int button = 0;
	
	//initiate the ultrasonic sensor
	private static final Port usPort = LocalEV3.get().getPort("S1");	// Because we don't bother to close this resource
    static SensorModes usSensor = new EV3UltrasonicSensor(usPort); 		// usSensor is the instance
    static SampleProvider usDistance = usSensor.getMode("Distance"); 	// usDistance provides samples from this instance
    static float[] usData = new float[usDistance.sampleSize()];		    // usData is the buffer in which data are returned
    
    //define the search location
    static final int LLx = 2;
    static final int LLy = 2;
    static final int URx = 5;
    static final int URy = 5;
    
    public static void main(String[] args) throws OdometerExceptions {
    	
    	int buttonChoice;
        Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
        Display odometryDisplay = new Display(textlcd);
        UltrasonicLocalizer uLocalizer = new UltrasonicLocalizer(odometer,usDistance, usData);    
        Test test = new Test(textlcd);
        
        //wait until the user decides which type of program to run
	    do {
	        // clear the display
	        textlcd.clear();
	        
	        // ask the user whether the navigation will be done with or without obstacles
	        textlcd.drawString("< Left | Right >", 0, 0);
	        textlcd.drawString("       |        ", 0, 1);
	        textlcd.drawString(" Color |Falling ", 0, 2);
	        textlcd.drawString(" Detec |  Edge  ", 0, 3);
	        textlcd.drawString(" -tion |        ", 0, 4);
	
	        buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
	        
	    }while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT); 
	    
	    //Rising edge localization is chosen
        if (buttonChoice == Button.ID_LEFT) {
            
        	//Start all relevant threads for rising edge localization
            Thread testThread = new Thread(test);
            testThread.start();
            button = 1;

        }
        
        //Falling edge localization is chosen
        if(buttonChoice == Button.ID_RIGHT) {
        	
        	//Start all relevant threads for falling edge localization
        	Thread odoThread = new Thread(odometer);
            odoThread.start();
            
            Thread odoDisplayThread = new Thread(odometryDisplay);
            odoDisplayThread.start();
            
            Thread uLocalizerThread = new Thread(uLocalizer);
            uLocalizerThread.start();
            
            button = 2;
            
        }
        
        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
		
    }
}