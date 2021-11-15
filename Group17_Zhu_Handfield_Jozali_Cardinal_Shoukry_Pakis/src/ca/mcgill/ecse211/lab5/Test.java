package ca.mcgill.ecse211.lab5;

import java.text.DecimalFormat;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * @author Maxime Cardinal
 * 
 *This class is responsible of the detection of the rings
 */

public class Test extends Thread{
	
	//initiate ring to find
	static String rightColor = "blue";
	static String foundColor;
	
	//initiate the sensor
	private static EV3ColorSensor lightSensor;
	private static Port lsPort = LocalEV3.get().getPort("S3");
	
	//initiate all mean values (normalized)
	static double blueRingRedMean = 0.2139;
	static double blueRingGreenMean = 0.6483;
	static double blueRingBlueMean = 0.7307;
	
	static double greenRingRedMean = 0.4814;
	static double greenRingGreenMean = 0.8662;
	static double greenRingBlueMean = 0.1340;
	
	static double yellowRingRedMean = 0.8624;
	static double yellowRingGreenMean = 0.4922;
	static double yellowRingBlueMean = 0.1176;
	
	static double orangeRingRedMean = 0.9626;
	static double orangeRingGreenMean = 0.2564;
	static double orangeRingBlueMean = 0.0879;
	
	//initiate samples 
	static double redSample = 0.0;
	static double greenSample = 0.0;
	static double blueSample = 0.0;
	
	//initiate normalized samples
	static double normalizedRedSample;
	static double normalizedGreenSample;
	static double normalizedBlueSample;	
	
	//initiate colors detection
	static double blueRingDetector;
	static double greenRingDetector;
	static double yellowRingDetector;
	static double orangeRingDetector;
	static double detectionRange = 0.15; //value has to be changed
	
	private static TextLCD textlcd;
	
	public Test(TextLCD textlcd2) {
		Test.textlcd = textlcd2;
	}

	public void run() {
		boolean keepGoing = true;
		lightSensor = new EV3ColorSensor(lsPort);
		lightSensor.setCurrentMode("RGB");
		SampleProvider colorSensor=lightSensor.getRGBMode();
		float[] rgb=new float[colorSensor.sampleSize()];
		
		while(keepGoing) {
			
			textlcd.clear();
			colorSensor.fetchSample(rgb, 0);
		    
			redSample = rgb[0];
			greenSample =  rgb[1];
			blueSample =  rgb[2];		
			
			//denominator calculation of the normalization
			double normalizationDivider = Math.sqrt(redSample*redSample+greenSample*greenSample+blueSample*blueSample);
			
			//normalize the samples
			normalizedRedSample = redSample / normalizationDivider;
			normalizedGreenSample = greenSample / normalizationDivider;
			normalizedBlueSample = blueSample / normalizationDivider;
			
			blueRingDetector = Math.sqrt(Math.pow((normalizedRedSample-blueRingRedMean), 2)
											+Math.pow((normalizedGreenSample-blueRingGreenMean), 2)
												+Math.pow((normalizedBlueSample-blueRingBlueMean), 2));
			
			greenRingDetector = Math.sqrt(Math.pow((normalizedRedSample-greenRingRedMean), 2)
											+Math.pow((normalizedGreenSample-greenRingGreenMean), 2)
												+Math.pow((normalizedBlueSample-greenRingBlueMean), 2));
			
			yellowRingDetector = Math.sqrt(Math.pow((normalizedRedSample-yellowRingRedMean), 2)
											+Math.pow((normalizedGreenSample-yellowRingGreenMean), 2)
												+Math.pow((normalizedBlueSample-yellowRingBlueMean), 2));
			
			orangeRingDetector = Math.sqrt(Math.pow((normalizedRedSample-orangeRingRedMean), 2)
											+Math.pow((normalizedGreenSample-orangeRingGreenMean), 2)
												+Math.pow((normalizedBlueSample-orangeRingBlueMean), 2));
			
			//check which ring has been detected
			if(blueRingDetector<detectionRange) {
				//blue ring is detected
				textlcd.drawString("Object detected", 0, 3);
		        textlcd.drawString("Blue", 0, 4);
		        Sound.beep();
			}
			else if(greenRingDetector<detectionRange) {
				//green ring is detected
				textlcd.drawString("Object detected", 0, 3);
		        textlcd.drawString("Green", 0, 4);
		        Sound.beep();
			}
			else if(yellowRingDetector<detectionRange) {
				//yellow ring is detected
				textlcd.drawString("Object detected", 0, 3);
		        textlcd.drawString("Yellow", 0, 4);
		        Sound.beep();
			}
			else if(orangeRingDetector<detectionRange) {
				//orange ring is detected
				textlcd.drawString("Object detected", 0, 3);
		        textlcd.drawString("Orange", 0, 4);
		        Sound.beep();
			}
			else {
				//no ring is detected, this part could be removed
			}
		}
	}
		
}
