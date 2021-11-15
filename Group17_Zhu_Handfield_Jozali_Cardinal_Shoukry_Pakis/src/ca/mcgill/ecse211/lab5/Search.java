package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class travels the robot through the search grid and localize rings
 * 
 * @author Maxime Cardinal
 */
public class Search extends Thread{
	
	double wheelRad = Lab5.WHEEL_RAD;
	double track = Lab5.TRACK;
	private static final int FORWARD_SPEED = 200;
    private static final int ROTATE_SPEED = 100;
	private static final double TILE_SIZE = 30.48;
	private static boolean keepGoing = true;
	
	//this light sensor is the right one
	private static EV3ColorSensor LightSensor;
	private static SampleProvider sensorVal;
	float[] sensorValData;
	
	//this light sensor is the left one
	private static EV3ColorSensor LightSensor2;
	private static SampleProvider sensorVal2;
	float[] sensorValData2;
	
	double xDistance = 0;
	double yDistance = 0;
	double angleBeta = 0;
	double distanceToTravel = 0;
	double finalAngle = 0;
	
	double currentCoordinates [] = new double[3];
	double currentXposition = 0;
	double currentYposition = 0;
	double currentAngle = 0;
	double initialAngle = 0;
	
	private Odometer odometer;
	
	public Search (Odometer odometer, EV3ColorSensor LightSensor, SampleProvider sensorVal, EV3ColorSensor LightSensor2, SampleProvider sensorVal2) {		
		this.odometer = odometer;	
		
		Search.LightSensor = LightSensor;
		Search.sensorVal = sensorVal;
	    this.sensorValData = new float[LightSensor.sampleSize()];
	    
	    Search.LightSensor2 = LightSensor2;
	    Search.sensorVal2 = sensorVal2;
	    this.sensorValData2 = new float[LightSensor2.sampleSize()];
	}
	
	public void run() {
		
		int xCounter = Lab5.URx - Lab5.LLx;
		int yCounter = Lab5.URy - Lab5.LLy;
		
		travelTo2(Lab5.LLx, Lab5.LLy, 0);
		positionCorrection(Lab5.LLx, Lab5.LLy);
		Sound.beepSequenceUp();
		//this.odometer.setXYT(Lab5.LLx*TILE_SIZE, Lab5.LLy*TILE_SIZE, 180*Math.atan2(Lab5.LLy, Lab5.LLx)/Math.PI);
		//search(xCounter, yCounter);	
		travelTo2(Lab5.URx,Lab5.URy,0);
		positionCorrection(Lab5.URx, Lab5.URy);
	}
	
	/**
	 * This method is responsible of the travel of the robot from a 
	 * point to another
	 * 
	 * @param x: corner position on the x-axis
	 * @param y: corner position on the y-axis
	 * @throws InterruptedException 
	 */
	void travelTo(double x, double y){
		
		currentCoordinates = this.odometer.getXYT();
		currentXposition = currentCoordinates[0];
		currentYposition = currentCoordinates[1];
		currentAngle = currentCoordinates[2];
		
		xDistance = x*TILE_SIZE - currentXposition;//distance in x to travel
		yDistance = y*TILE_SIZE - currentYposition;//distance in y to travel
		
		angleBeta = Math.atan2(xDistance, yDistance); //calculate the angle where the next point is	
		finalAngle = angleBeta - currentAngle*Math.PI/180;
		
		if(finalAngle<-Math.PI) {
			finalAngle = finalAngle + 2*Math.PI;
		}
		else if(finalAngle>Math.PI){
			finalAngle = finalAngle - 2*Math.PI;
		}
		turnTo(finalAngle);	
		
		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		distanceToTravel = Math.hypot(xDistance,yDistance);//calculate the hypotenuse
		
		this.odometer.leftMotor.setSpeed(FORWARD_SPEED);
		this.odometer.rightMotor.setSpeed(FORWARD_SPEED);

	    this.odometer.leftMotor.rotate(convertDistance(wheelRad, distanceToTravel), true);
	    this.odometer.rightMotor.rotate(convertDistance(wheelRad, distanceToTravel), false);
		
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

	    this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, theta*180/Math.PI), true);
	    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, theta*180/Math.PI), false);
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
	 * This method is responsible of the traveling of the robot
	 * through the search grid
	 * 
	 * @param xCounter: length of the search grid
	 * @param yCounter: height of the search grid
	 */
	void search(int xCounter, int yCounter) {
		
		for(int i = 0; i < xCounter ; i++ ) {
			
			//move up
			if(i%2 == 0) {		
				travelTo2((Lab5.LLx+i+1) , Lab5.URy, i);	
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
				}
				positionCorrection((Lab5.LLx+i+1), Lab5.URy);
			}
			
			//move down
			else if(i%2 != 0) {	
				travelTo2((Lab5.LLx +i+1) , (Lab5.LLy), i);
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
				}
				positionCorrection((Lab5.LLx+i+1), Lab5.LLy);
			}	
		}
	}
	
	/**
	 * This method is responsible of correction the position of the robot
	 * after each travel
	 */
	//this method is not done yet, I am working on it atm
	void positionCorrection(int xCorrected, int yCorrected) {
		
		currentCoordinates = this.odometer.getXYT();
		initialAngle = currentCoordinates[2];
		
		this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
		this.odometer.rightMotor.setSpeed(ROTATE_SPEED);
		
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
		
	    //turn 90 degress to the left
	    this.odometer.leftMotor.rotate(-convertAngle(wheelRad, track, 95.0), true);
	    this.odometer.rightMotor.rotate(convertAngle(wheelRad, track, 95.0), false);
	    
	    //reinitialize the position
	    this.odometer.setXYT(xCorrected*TILE_SIZE, yCorrected*TILE_SIZE, 0);	    
	    
	}
	
	void travelTo2(double x, double y, int XCounter) {

		currentCoordinates = this.odometer.getXYT();
		currentXposition = currentCoordinates[0];
		currentYposition = currentCoordinates[1];

		xDistance = x * TILE_SIZE - currentXposition;// distance in x to travel
		yDistance = y * TILE_SIZE - currentYposition;// distance in y to travel
		
		if(XCounter%2 == 0) {
		
			this.odometer.leftMotor.setSpeed(FORWARD_SPEED);
			this.odometer.rightMotor.setSpeed(FORWARD_SPEED);
			
			this.odometer.leftMotor.rotate(convertDistance(wheelRad, yDistance), true);
			this.odometer.rightMotor.rotate(convertDistance(wheelRad, yDistance), false);
			
			this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
			this.odometer.rightMotor.setSpeed(ROTATE_SPEED);
			
			this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, 90), true);
		    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, 90), false);
			
		    this.odometer.leftMotor.setSpeed(FORWARD_SPEED);
			this.odometer.rightMotor.setSpeed(FORWARD_SPEED);

			this.odometer.leftMotor.rotate(convertDistance(wheelRad, xDistance), true);
			this.odometer.rightMotor.rotate(convertDistance(wheelRad, xDistance), false);
			
			this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
			this.odometer.rightMotor.setSpeed(ROTATE_SPEED);
			
			this.odometer.leftMotor.rotate(-convertAngle(wheelRad, track, 90), true);
		    this.odometer.rightMotor.rotate(convertAngle(wheelRad, track, 90), false);
		    
		}
		
		else {
			
			this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
			this.odometer.rightMotor.setSpeed(ROTATE_SPEED);
			
			this.odometer.leftMotor.rotate(convertAngle(wheelRad, track, 180), true);
		    this.odometer.rightMotor.rotate(-convertAngle(wheelRad, track, 180), false);
			
			this.odometer.leftMotor.setSpeed(FORWARD_SPEED);
			this.odometer.rightMotor.setSpeed(FORWARD_SPEED);
			
			this.odometer.leftMotor.rotate(-convertDistance(wheelRad, yDistance), true);
			this.odometer.rightMotor.rotate(-convertDistance(wheelRad, yDistance), false);
			
			this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
			this.odometer.rightMotor.setSpeed(ROTATE_SPEED);
			
			this.odometer.leftMotor.rotate(-convertAngle(wheelRad, track, 90), true);
		    this.odometer.rightMotor.rotate(convertAngle(wheelRad, track, 90), false);
			
		    this.odometer.leftMotor.setSpeed(FORWARD_SPEED);
			this.odometer.rightMotor.setSpeed(FORWARD_SPEED);

			this.odometer.leftMotor.rotate(convertDistance(wheelRad, xDistance), true);
			this.odometer.rightMotor.rotate(convertDistance(wheelRad, xDistance), false);
			
			this.odometer.leftMotor.setSpeed(ROTATE_SPEED);
			this.odometer.rightMotor.setSpeed(ROTATE_SPEED);
			
			this.odometer.leftMotor.rotate(-convertAngle(wheelRad, track, 90), true);
		    this.odometer.rightMotor.rotate(convertAngle(wheelRad, track, 90), false);
			
		}
	}
}
