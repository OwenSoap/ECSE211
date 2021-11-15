package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import java.util.concurrent.TimeUnit;
import ca.mcgill.ecse211.lab5.Lab5;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class corrects the initial angle of the robot to 0-degree
 * 
 * @author Maxime Cardinal
 */
public class UltrasonicLocalizer extends Thread{
	
	private SampleProvider us;
	private float[] usData;
	private Odometer odometer;
	
	double wheelRad = Lab5.WHEEL_RAD;
	double track = Lab5.TRACK;
	private static final int ROTATE_SPEED = 100;
	private static final int fullRotation = 360;
	private static final int halfRotation = 180;
	private static final int DISTANCE_RANGE = 40;
	private static boolean keepGoing = true;
	
	double currentCoordinates [] = new double[3];
	double angleBeta;
	double currentAngle;
	double finalAngle;
	
	private static double firstFallingEdge;
	private static double secondFallingEdge;
	double previousReading [] = new double[2];
	
	//constructor
	public UltrasonicLocalizer(Odometer odometer, SampleProvider us, float[] usData){ 
		this.odometer = odometer;
		this.us = us;
	    this.usData = usData;
	}
	
	//starts the localisation
	public void run() {

		if(Lab5.button == 2) {
			fallingEdge();	
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
			}
		}
		
		LightLocalizer lightLocalizer = new LightLocalizer(odometer);
		Thread lightLocalizerThread= new Thread(lightLocalizer);
        lightLocalizerThread.start();
        
	}
	
	/**
	 * This method is responsible for localising the robot's orientation
	 * using falling edge
	 */
	void fallingEdge() {
		previousReading[0] = 0;
		previousReading[1] = 0;
		
		this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
		this.odometer.rightMotor.setSpeed(ROTATE_SPEED);
		
		//if the robot is facing a wall, turn around befor computing
		int distance;
		us.fetchSample(usData, 0); // acquire data
        distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
        if(distance<DISTANCE_RANGE) {
    		Sound.beepSequence();
    		this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, halfRotation), true);
    	    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, halfRotation), false);
    	    this.odometer.setXYT(0, 0, 0);
        }
		
        //turn right until the robot sees the back wall
		this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, fullRotation), true);
	    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, fullRotation), true);
	    
	    while(keepGoing){
	    	us.fetchSample(usData, 0); // acquire data
	        distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
	        if(distance<DISTANCE_RANGE){
	        	if (distance<previousReading[0] && distance<previousReading[1]) {
					Sound.beep();
					this.odometer.leftMotor.stop(true);
					this.odometer.rightMotor.stop(false);
					currentCoordinates = this.odometer.getXYT();
					firstFallingEdge = currentCoordinates[2];
					keepGoing = false;
				}
	        	else {
	        		previousReading[1] = previousReading[0];
	        		previousReading[0] = distance;

	        	}
	        }
	    }
	    
	    keepGoing = true;
	    previousReading[0] = 0;
		previousReading[1] = 0;
		
		//turn left until the robot sees the left wall
		this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
		this.odometer.rightMotor.setSpeed(ROTATE_SPEED);
		
		this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, -fullRotation), true);
	    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, -fullRotation), true);
	    
	    try {
			TimeUnit.SECONDS.sleep(2);
		} catch (InterruptedException e) {
		}
	    
	    while(keepGoing){
	    	us.fetchSample(usData, 0); // acquire data
	        distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
	        if(distance<DISTANCE_RANGE){
	        	if (distance<previousReading[0] && distance<previousReading[1]) {
					Sound.beep();
					this.odometer.leftMotor.stop(true);
					this.odometer.rightMotor.stop(false);
					currentCoordinates = this.odometer.getXYT();
					secondFallingEdge = currentCoordinates[2];
					keepGoing = false;
				}
	        	else {
	        		previousReading[1] = previousReading[0];
	        		previousReading[0] = distance;

	        	}
	        }
	    }
	    
	    //angle algorithm
	    if(firstFallingEdge<secondFallingEdge) {
	    	angleBeta =45- (firstFallingEdge+secondFallingEdge)/2;
	    }
	    else if(firstFallingEdge>secondFallingEdge) {
	    	angleBeta =225-(firstFallingEdge+secondFallingEdge)/2;
	    }
	    currentAngle = currentCoordinates[2];
	    finalAngle = 180 - (currentAngle + angleBeta);
		turnTo(finalAngle);
		turnTo(0);
		
	}
	
	/**
	 * This method is responsible for adjusting the angle of the robot
	 * toward the next position it needs to travel
	 * 
	 * @param theta: angle, in radian, to turn the robot to
	 */
	void turnTo(double theta) {
		this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
		this.odometer.rightMotor.setSpeed(ROTATE_SPEED);

	    this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, theta), true);
	    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, theta), false);
	}
	
	/**
	 * This method converts a distance to the total rotation that each
	 * wheel needs to perform
	 * 
	 * @param radius: wheel radius
	 * @param distance: distance to travel
	 * @return: integer
	 */
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method convert an angle to the total rotation that
	 * each wheel needs to perform
	 * 
	 * @param radius: wheel radius
	 * @param width: wheel base of the robot
	 * @param angle: the next robot orientation
	 * @return: convertDistance()
	 */
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}
