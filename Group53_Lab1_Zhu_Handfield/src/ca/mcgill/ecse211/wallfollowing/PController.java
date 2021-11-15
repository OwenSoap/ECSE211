package ca.mcgill.ecse211.wallfollowing;

import javax.swing.plaf.synth.SynthSpinnerUI;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements the PController style the Wall Follower for Lab1 on the EV3 Platform
 * THe PController dynamically changes its adjustment speed based on the size of the error
 * @author yizhu
 *
 */
public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 20;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	/**
	 * This method initializes the motors and sets their default speeds
	 * along with defining all the other relevant values for this version of the PController such as center and width
	 * it also activates the robot to move forward
	 * @param bandCenter
	 * @param bandwidth
	 */
	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	/**
	 * this method requires the current distance of the robot from the wall
	 * it then takes that distance and runs it through a filter to ensure it is an accurate reading 
	 * should it be accurate, it calculates a dynamic adjustment ratio given the distance from the wall
	 * it then moves the robot correspondingly away or back towards the objective zone
	 * turning the right wheel faster than the left turns the robot right and vice-versa
	 */
	@Override
	public void processUSData(int distance) {
		
		// TODO: process a movement based on the us distance passed in (P style)

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// TODO: process a movement based on the us distance passed in (P style)		
		if (this.distance > 2000) //capping distance to prevent too large of an integer turning into negative number
			this.distance = 230; //resetting the distance to create manageable variables (prevent int -> negative float)
			//the shifted distance is tested and larger than any sample distance, but not so large as to be damaging
		
		//PController specifies that within each case, speed at which turn is effected is distance dependent
		int errorMultiplier = 4; //a tested and appropriate multiplier by which the robot moves depending on error
		//create a distance dependent multiplier by which to modify turning speeds
		int errorCorrection = errorMultiplier * Math.abs(this.distance - this.bandCenter);
		
		if (errorCorrection > 200 ) //capping the error correction to prevent overly drastic and possibly harmful adjustment
			errorCorrection = 200; 
		if (this.distance <= (this.bandCenter + this.bandWidth)
				&& this.distance >= (this.bandCenter - this.bandWidth)) { //the robot is within the specified area
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); //identical motor speed to maintain straight
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		else if (this.distance < 10) { //the robot is dangerously close to the wall
			WallFollowingLab.rightMotor.setSpeed(500); //rapidly move away from the potential collision 
			WallFollowingLab.leftMotor.setSpeed(250);
			WallFollowingLab.leftMotor.backward();
			WallFollowingLab.rightMotor.backward();
		} 
		else if (this.distance < 30) { //the robot is too close, but not significantly, to the wall
			WallFollowingLab.leftMotor.setSpeed((int) (2.5 * MOTOR_SPEED)); //move away relatively quickly
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); 
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.backward();
		}
		else if (this.distance < (this.bandCenter - this.bandWidth)) { //the robot is out of the specified zone
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + errorCorrection); //relative to how close it is, move away 
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - errorCorrection / (errorMultiplier));
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		else { //the robot is too far away from the wall, depending on how far off, move back towards the desired zone
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + errorCorrection  / (errorMultiplier));
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - errorCorrection  / (errorMultiplier));
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}

	/**
	 * this method simply polls for the next distance of the robot to repeat the above process
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
