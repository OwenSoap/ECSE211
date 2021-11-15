package ca.mcgill.ecse211.lab3;
import java.text.DecimalFormat;
import lejos.hardware.lcd.TextLCD;

public class OdometryDisplay extends Thread {
	private static final long DISPLAY_PERIOD = 250;
	private Odometer odometer;
	private TextLCD t;
	private UltrasonicController cont;

	// constructor
	public OdometryDisplay(Odometer odometer, TextLCD t, UltrasonicController cont) {
		this.odometer = odometer;
		this.t = t;
		this.cont = cont;
	}

	// run method (required for Thread)
	public void run() {
		long displayStart, displayEnd;
		double[] position = new double[3];

		// clear the display once
		t.clear();

		while (true) {
			displayStart = System.currentTimeMillis();

			// clear the lines for displaying odometry information
			t.drawString("X:              ", 0, 0);
			t.drawString("Y:              ", 0, 1);
			
			// get the odometry information
			odometer.getPosition(position, new boolean[] { true, true, true });
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			// display odometry information
			for (int i = 0; i <= 2; i++) {
				t.drawString(numberFormat.format(position[i]), 3, i);
			}
			t.drawString("T:"+ numberFormat.format(odometer.getThetaDisp()), 0, 2);
	
			// throttle the OdometryDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that OdometryDisplay will be interrupted
					// by another thread
				}
			}
		}
	}

}