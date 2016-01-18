package de.tuhh.diss.plotbot;

import lejos.nxt.Button;
import lejos.nxt.LCD;

public class Plotbot {
	public static void main(String[] args)
	{
		LCD.drawString("Hello", 0, 0);
		
		try {
			new PlotRectangle(20,true);
		} catch (MotorsHasBeenStoppedException e) {
			LCD.drawString("Motors has been stopped!", 0, 2);
		}
		
		LCD.drawString("DONE", 0, 1);
		Button.ESCAPE.waitForPressAndRelease();
	}
}