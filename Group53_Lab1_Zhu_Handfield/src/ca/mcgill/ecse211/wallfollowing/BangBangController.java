package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;
/**
 * this class implements the BangBang type Wall Follower for Lab1 on the EV3 platform
 * the BangBang type follows a wall with constant speeds of adjustment, regardless of how significant the error is
 * @author yizhu
 *
 */
public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
/**
 * this method is the default constructor for the BangBang type wall follower
 * it takes in all the relevant motor adjustment speeds and desired zone measurements to remain within
 * turning the right wheel faster than the left turns the robot right and vice-versa
 * @param bandCenter
 * @param bandwidth
 * @param motorLow
 * @param motorHigh
 */
	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	/**
	 * this method takes in the distance of the robot from the wall 
	 * it then turns it away if too close and turn it back towards the wall if too far
	 * the speeds at which these adjustments happen are predefined and fixed regardless of the error
	 */
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		if (this.distance > 2000) // capping distance to prevent too large of an integer turning into negative
			// number, tested to prevent too large int turning into negative float
			this.distance = 230; // resetting the distance to create manageable variables
		
		int error = this.distance - this.bandCenter; // difference between distance and center
		int miner = this.bandCenter - this.bandwidth; // minimum acceptable distance
		int maxer = this.bandCenter + this.bandwidth; // maximum acceptable distance
		
		//all turning speeds per case are constant regardless of distance within each from wall as per specification
		if (Math.abs(error) <= this.bandwidth) { //robot is within the desired distance from the wall 
			WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		else if (this.distance < 10) { //the robot is dangerously too close to the wall
			WallFollowingLab.rightMotor.setSpeed(motorHigh * 3); //back away from the wall immediately
			WallFollowingLab.leftMotor.setSpeed(motorHigh * 2); //wheels at different speed
			WallFollowingLab.leftMotor.backward(); //different speed slightly turns the robot, so as to better adjust trajectory
			WallFollowingLab.rightMotor.backward();
		} 
		else if (distance - miner < 0) { // the robot is too close to the wall
			WallFollowingLab.leftMotor.setSpeed(motorHigh * 3); //simply turn back right, away from the wall
			WallFollowingLab.rightMotor.setSpeed(motorLow); //speed of right higher to increase avoidance of collision
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		} 
		else if (distance - maxer > 0) { // the robot is too far from the wall
			WallFollowingLab.leftMotor.setSpeed(motorLow); //gently turn back towards wall
			WallFollowingLab.rightMotor.setSpeed(motorHigh); //speeds lower so as to prevent over correction
			WallFollowingLab.leftMotor.forward(); 
			WallFollowingLab.rightMotor.forward();
		}
	}
	/**
	 * this method polls for the distance of the robot from the wall 
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
