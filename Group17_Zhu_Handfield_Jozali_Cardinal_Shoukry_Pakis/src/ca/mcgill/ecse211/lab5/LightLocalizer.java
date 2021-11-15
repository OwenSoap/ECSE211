package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class centers the robot at the point (0,0) of the grid
 * 
 * @author Maxime Cardinal
 */
public class LightLocalizer extends Thread{
	
	//this light sensor is the right one
	private Port lsPort = LocalEV3.get().getPort("S2");
	private static EV3ColorSensor LightSensor;
	private static SampleProvider sensorVal;
	float[] sensorValData;
	
	//this light sensor is the left one
	private Port lsPort4 = LocalEV3.get().getPort("S4");//new light sensor from port 4
	private static EV3ColorSensor LightSensor2;
	private static SampleProvider sensorVal2;
	float[] sensorValData2;
	
	private static boolean keepGoing = true;
	private Odometer odometer;
	
	double wheelRad = Lab5.WHEEL_RAD;
	double track = Lab5.TRACK;
	private static final int MOVING_SPEED = 100;
	double currentCoordinates [] = new double[3];
	double initialAngle = 0;
	
	/**
	* This is the default class constructor. An existing instance of the odometer is used. This is to
    * ensure thread safety.
    * 
    * @throws OdometerExceptions
    */
    public LightLocalizer(Odometer odometer){
    	
	    this.odometer = odometer;
	    LightLocalizer.LightSensor = new EV3ColorSensor(lsPort);
	    LightLocalizer.sensorVal = LightSensor.getRedMode();
	    this.sensorValData = new float[LightSensor.sampleSize()];
	    
	    LightLocalizer.LightSensor2 = new EV3ColorSensor(lsPort4);
	    LightLocalizer.sensorVal2 = LightSensor2.getRedMode();
	    this.sensorValData2 = new float[LightSensor2.sampleSize()];
	    
    }
    
    public void run() {
    	this.odometer.leftMotor.setSpeed(MOVING_SPEED);
		this.odometer.rightMotor.setSpeed(MOVING_SPEED);
		
		this.odometer.leftMotor.forward();
		this.odometer.rightMotor.forward();
		
		while (keepGoing) {
			
			sensorVal.fetchSample(sensorValData, 0);
			if(sensorValData[0]<0.35) {
				Sound.beep();
				this.odometer.leftMotor.stop(true);
				this.odometer.rightMotor.stop(false);
				currentCoordinates = this.odometer.getXYT();
				keepGoing = false;
			}
		}
		
		this.odometer.leftMotor.rotate(-convertDistance(wheelRad,5), true);
	    this.odometer.rightMotor.rotate(-convertDistance(wheelRad,5), false);
	    
	    this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, 90.0), true);
	    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, 90.0), false);
	    
	    this.odometer.leftMotor.forward();
		this.odometer.rightMotor.forward();
	    
		keepGoing = true;
		while (keepGoing) {
			
			sensorVal.fetchSample(sensorValData, 0);
			if(sensorValData[0]<0.35) {
				Sound.beep();
				this.odometer.leftMotor.stop(true);
				this.odometer.rightMotor.stop(false);
				keepGoing = false;
			}
		}
		
		this.odometer.leftMotor.rotate(-convertDistance(wheelRad,5), true);
	    this.odometer.rightMotor.rotate(-convertDistance(wheelRad,5), false);
		
		this.odometer.leftMotor.rotate(-convertAngle(wheelRad, track, 90.0), true);
	    this.odometer.rightMotor.rotate(convertAngle(wheelRad, track, 90.0), false);
	    
	    Sound.beepSequenceUp();
	    
	    try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {

		}
	    
	    this.odometer.setXYT(0, 0, 0);
	    positionCorrection(0,0);
	    Search search = new Search(odometer, LightSensor, sensorVal, LightSensor2, sensorVal2);
	    Thread searchThread = new Thread(search);
        searchThread.start();
	    
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
	
	/**
	 * This method is responsible of correction the position of the robot
	 * after each travel
	 */
	//this method is not done yet, I am working on it atm
	void positionCorrection(int xCorrected, int yCorrected) {
		
		currentCoordinates = this.odometer.getXYT();
		initialAngle = currentCoordinates[2];
		
		//turn to 0 degree
		turnTo((0-currentCoordinates[2]*Math.PI/180));
		
		this.odometer.leftMotor.setSpeed(MOVING_SPEED);
		this.odometer.rightMotor.setSpeed(MOVING_SPEED);
		
		this.odometer.leftMotor.forward();
		this.odometer.rightMotor.forward();
		
		keepGoing = true;
		while (keepGoing) {
			
			sensorVal.fetchSample(sensorValData, 0);
			sensorVal2.fetchSample(sensorValData2, 0);
			if(sensorValData[0]<0.35 || sensorValData2[0]<0.35) {
				Sound.beep();
				this.odometer.leftMotor.stop(true);
				this.odometer.rightMotor.stop(false);
				
				//right wheel crossed first
				if(sensorValData[0]<0.35) {
					keepGoing = true;
					sensorVal2.fetchSample(sensorValData2, 0);
					
					//if it is already alligned
					if(sensorValData2[0]<35) {
						Sound.beep();
						keepGoing = false;
					}
					else {
						while(keepGoing) {
							this.odometer.leftMotor.forward();
							sensorVal2.fetchSample(sensorValData2, 0);
							if(sensorValData2[0]<0.35) {
								Sound.beep();
								this.odometer.leftMotor.stop(false);
								keepGoing = false;
							}
						}
					}
				}
				
				//left wheel crossed first
				if(sensorValData2[0]<0.35) {
					keepGoing = true;
					sensorVal.fetchSample(sensorValData, 0);
					
					//if it is already alligned
					if(sensorValData[0]<35) {
						Sound.beep();
						keepGoing = false;
					}
					else {
						while(keepGoing) {
							this.odometer.rightMotor.forward();
							sensorVal.fetchSample(sensorValData, 0);
							if(sensorValData[0]<0.35) {
								Sound.beep();
								this.odometer.rightMotor.stop(false);
								keepGoing = false;
							}
						}
					}	
				}			
			}
		}
		
		//correct the offset of the sensors
		this.odometer.leftMotor.rotate(-convertDistance(wheelRad,5), true);
	    this.odometer.rightMotor.rotate(-convertDistance(wheelRad,5), false);
		
	    //turn 90 degress to the right
	    this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, 90.0), true);
	    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, 90.0), false);
	    
	    //move bacward a little bit for protection
	    this.odometer.leftMotor.rotate(-convertDistance(wheelRad,5), true);
	    this.odometer.rightMotor.rotate(-convertDistance(wheelRad,5), false);
		
		this.odometer.leftMotor.forward();
		this.odometer.rightMotor.forward();
		
		//this stops when the sensors reach the line
		keepGoing = true;
		while(keepGoing) {
			sensorVal.fetchSample(sensorValData, 0);
			sensorVal2.fetchSample(sensorValData2, 0);
			if(sensorValData[0]<0.35 || sensorValData2[0]<0.35) {
				Sound.beep();
				this.odometer.leftMotor.stop(true);
				this.odometer.rightMotor.stop(false);
				keepGoing = false;
			}	
		}
		
		//correct the offset of the sensors
		this.odometer.leftMotor.rotate(-convertDistance(wheelRad,5), true);
	    this.odometer.rightMotor.rotate(-convertDistance(wheelRad,5), false);
		
	    //turn 90 degress to the left
	    this.odometer.leftMotor.rotate(-convertAngle(wheelRad, track, 90.0), true);
	    this.odometer.rightMotor.rotate(convertAngle(wheelRad, track, 90.0), false);
	    
	    //reinitialize the position
	    this.odometer.setXYT(xCorrected, yCorrected, 0);	    
	    
	}
	
	/**
	 * This method is responsible for adjusting the angle of the robot
	 * toward the next position it needs to travel
	 * 
	 * @param theta: angle, in radian, to turn the robot to
	 */
	void turnTo(double theta) {
		this.odometer.leftMotor.setSpeed(MOVING_SPEED);
		this.odometer.rightMotor.setSpeed(MOVING_SPEED);

	    this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, theta*180/Math.PI), true);
	    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, theta*180/Math.PI), false);
	}
	
}
