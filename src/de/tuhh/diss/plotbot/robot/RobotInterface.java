package de.tuhh.diss.plotbot.robot;

import de.tuhh.diss.plotbot.exceptions.MotorException;
import de.tuhh.diss.plotbot.exceptions.OutOfWorkspaceException;

public interface RobotInterface { //TODO: Brauchen wir das Interface ueberhaupt ? 
	
	public RobotInterface getPhysicalRobot();
	public boolean calibrateMotors();
	public void stopAllMotors();
	public void movePenTo(int xTarget, int yTarget) throws OutOfWorkspaceException;
	public void movePenToInSteps(int xTarget, int yTarget, int steps, int style) throws OutOfWorkspaceException;
	
	/////** ARM **/////
	public int getArmLength();
	public int getArmMinAngle();
	public int getArmMaxAngle();
	public double getArmAngle();
	public double getArmRotationSpeed();
	public void setArmSpeed(int speed) throws IndexOutOfBoundsException;
	public void moveArmTo(double angle) throws OutOfWorkspaceException;
	public void moveArmTo(double armAngle, boolean immediateReturn) throws OutOfWorkspaceException;
	public void waitForArm();
	public void stopArm();
	
	/////** PEN **/////	
	public void setPen(boolean IO);
	public void stopPen();
	
	/////** Wheels **/////	
	public int getMaxFeed();
	public double getYCenter();
	public void setWheelSpeed(double speed) throws IndexOutOfBoundsException;
	public void moveWheels(double distance) throws OutOfWorkspaceException;
	public void moveWheels(double distance, boolean immediateReturn) throws OutOfWorkspaceException;
	public void moveWheelsForward();
	public void moveWheelsBackward();
	public void waitForWheels();
	public void stopWheels();	
}