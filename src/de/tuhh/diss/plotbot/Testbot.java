package de.tuhh.diss.plotbot;

import de.tuhh.diss.plotbot.exceptions.MotorException;
import de.tuhh.diss.plotbot.exceptions.OutOfWorkspaceException;
import de.tuhh.diss.plotbot.robot.PhysicalRobot;
import de.tuhh.diss.plotbot.robot.RobotInterface;
import de.tuhh.diss.plotbot.robot.*;
import lejos.nxt.Button;
import lejos.nxt.LCD;

public class Testbot {
	RobotInterface robot;
	DrawControlChris drawChris;
	
	
	Testbot(){		// hier ist n arsch voll platz fuer testkram (einfach in main "Testbot" statt "Plotbot" aufrufen)
		
		LCD.drawString("Hello", 0, 0);
		

		robot = PhysicalRobot.ROBOT;
		robot.calibrateMotors();
		
		
		try {
			//CHRIS
//			drawChris.movePenTo(0, 50);
//			robot.setPen(true);
//			drawChris.movePenToInStepsV3(-50, 50, 5);
//			robot.setPen(false);
//			drawChris.movePenTo(0, 80);
//			robot.setPen(true);
//			drawChris.movePenToInStepsV2(-50, 80, 5);
//			robot.setPen(false);
//			drawChris.movePenTo(0, 110);
//			robot.setPen(true);
//			drawChris.movePenToInStepsV1(-50, 110, 5);
//			robot.setPen(false);
			
			//JULIUS
			robot.movePenTo(0, 50);
			robot.setPen(true);
			
			LCD.drawString("Julius1", 0, 6);
			Button.ENTER.waitForPressAndRelease();
			PhysicalRobot.ROBOT.movePenJulius1(-50, 0);
			LCD.drawString("done1", 0, 6);
			Button.ENTER.waitForPressAndRelease();
			robot.setPen(false);
			robot.movePenTo(0, 80);
			robot.setPen(true);
			
			LCD.drawString("Julius2", 0, 6);
			Button.ENTER.waitForPressAndRelease();
			PhysicalRobot.ROBOT.movePenJulius2(-50, 10);
			LCD.drawString("done2", 0, 6);
			Button.ENTER.waitForPressAndRelease();
			robot.setPen(false);
			robot.movePenTo(0, 110);
			robot.setPen(true);

			LCD.drawString("Julius3", 0, 6);
			Button.ENTER.waitForPressAndRelease();
			PhysicalRobot.ROBOT.movePenJulius3(-50, 10);
			LCD.drawString("done3", 0, 6);
			Button.ENTER.waitForPressAndRelease();
			robot.setPen(false);
			robot.movePenTo(0, 140);
			robot.setPen(true);
			
			LCD.drawString("Julius4", 0, 6);
			Button.ENTER.waitForPressAndRelease();
			PhysicalRobot.ROBOT.movePenJulius4(-50, 10);
			LCD.drawString("done4", 0, 6);
			Button.ENTER.waitForPressAndRelease();
			robot.setPen(false);
//			
//			//LENNART
//			try {
//				robot.movePenToLennart(0, 50);
//				robot.setPen(true);
//				robot.movePenHorizontalLennart(0, 50, -50, 10);
//				robot.setPen(false);
//				robot.setPen(true);
//				robot.movePenVerticalLennart(-50, 50, 50);
//				robot.setPen(false);
//				robot.setPen(true);
//				robot.movePenHorizontalLennart(-50, 100, 50, 10);
//				robot.setPen(false);
//				robot.setPen(true);
//				robot.movePenVerticalLennart(0, 100, -50);
//				robot.setPen(false);
//			} catch (MotorException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//			
		} catch (OutOfWorkspaceException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
			

			// TODO Auto-generated catch block

		

		
		Button.ESCAPE.waitForPressAndRelease();
	}
}
