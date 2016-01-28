package de.tuhh.diss.plotbot.robot;

import de.tuhh.diss.plotbot.exceptions.OutOfWorkspaceException;
import de.tuhh.diss.plotbot.utilities.Calc;
import lejos.nxt.LCD;
import lejos.util.Delay;



public class PhysicalRobot implements RobotInterface{

	///////////////////////////////////////////////////////
	//	VARIABLES
	///////////////////////////////////////////////////////
	public static final PhysicalRobot ROBOT = new PhysicalRobot();
	
	private final ArmModule ARM;
	private final PenModule PEN;
	private final WheelsModule WHEELS;
	
	
	///////////////////////////////////////////////////////
	//	METHODS
	///////////////////////////////////////////////////////	
	
	private PhysicalRobot(){
			PEN = new PenModule();
			ARM = new ArmModule();
			WHEELS = new WheelsModule();
	}
	
	public RobotInterface getPhysicalRobot(){
		return ROBOT;
	}
	
	/** Calls the calibration methods of every module
	 *  
	 *  @return true if calibration succeeded, false if calibration failed
	 */
	public boolean calibrateMotors(){
		try{
			PEN.calibrateMotorPen();
			ARM.calibrateMotorArm();
			WHEELS.calibrateMotorWheels();
			return true;
			
		} catch (OutOfWorkspaceException e) {
			stopAllMotors();
			return false;
		}
	}
	
	/** Stops all motors at once
	 * 
	 */
	public void stopAllMotors(){
		stopArm();
		stopPen();
		stopWheels();
	}
	
	/** Moves the pen to a certain position 
	 *  by moving arm and wheels at the same time continously
	 * 
	 * @param xTarget x coordinate of the target
	 * @param yTarget y coordinate of the target
	 */
	public void movePenTo(int xTarget, int yTarget) throws OutOfWorkspaceException{
		double yCenterToPen = Calc.getYCenterToPen(getArmLength(), getArmAngle());
		double yCenterOfRobot = getYCenter();
		double distanceToTravel = yTarget - yCenterOfRobot - yCenterToPen;
		
		moveArmTo((int)Calc.getAnglePen(getArmLength(), xTarget), true);
		moveWheels(distanceToTravel);
	}
	
	
	/** Moves the pen to a certain target in steps
	 *  by using small steps the pen will move in a straight line (in theory :P)
	 *  
	 * @param xTarget target x-position of pen
	 * @param yTarget target y-position of pen
	 * @param steps the amount of steps the movement shall be divided in
	 * @throws OutOfWorkspaceException
	 */
	public void movePenToInSteps(int xStart, int yStart, int xTarget, int yTarget, int steps) throws OutOfWorkspaceException{
		int startAngle =(int) Math.round(Calc.getAnglePen(getArmLength(), xStart));
		int endAngle =(int) Math.round(Calc.getAnglePen(getArmLength(), xTarget));
		int angleDifference = endAngle - startAngle;
		double angleStep = angleDifference / steps;		
		double fromAngle = startAngle;
		double toAngle = startAngle + angleStep;
		double timePerStep = Math.abs(angleStep / getArmRotationSpeed());
		double yDevianceAngle;
		double yDevianceStep = (yTarget - yStart) / steps; 
		double yStep;
		double necessaryWheelspeed;
		double wheelAngle;
		
		for(int it = 0; it < steps; it++){
			//TODO: Fix this wenn winkel rechts von 90 dann falsches vorzeichen
			yDevianceAngle = Calc.getYCenterToPen(getArmLength(), fromAngle) - Calc.getYCenterToPen(getArmLength(), toAngle);
			yStep = yDevianceAngle + yDevianceStep;
			
			wheelAngle = Math.round((yStep*360/(56*Math.PI)));
			//LCD.drawString("wheelA: " + String.valueOf(wheelAngle*84), 0, 4);
			necessaryWheelspeed = wheelAngle / timePerStep;
			LCD.drawString("motspd: " + String.valueOf(Math.round(necessaryWheelspeed)*84), 0, 4);
			
			moveArmTo(toAngle, true);
			//setWheelSpeed((int) Math.round(necessaryWheelspeed));
			LCD.drawString("yStep: " + String.valueOf(yStep), 0, 5);
			
			moveWheels(yStep);
			waitForArm();
			waitForWheels();
			
			fromAngle = toAngle;
			toAngle = toAngle + angleStep;
		}
	}
	
	// Julius Versuch
	
	/** Keine Steps. Durch Funktion direkte Geschwindigkeitsanpassung
	 */
	public void movePenJulius1(int xTarget, int yTarget) throws OutOfWorkspaceException{
		double startAngle = ARM.getAngle();
		double endAngle = Calc.getAnglePen(ArmModule.ARMLENGTH, xTarget);
		double yNow;
		double yDeviance;
		double yDevianceAngle;
		double ySum;
		double necessaryWheelspeed;
				
				yNow = WHEELS.getYCenter() + Calc.getYCenterToPen(ArmModule.ARMLENGTH, startAngle);
				yDeviance = yTarget - yNow;
				yDevianceAngle = Calc.getYCenterToPen(ArmModule.ARMLENGTH, startAngle) - Calc.getYCenterToPen(getArmLength(), endAngle);
				ySum = yDevianceAngle + yDeviance;
				
				moveArmTo(endAngle, true);
				moveWheels(ySum, true);
				
				do{
					necessaryWheelspeed = ARM.getRotationSpeed()*ArmModule.ARMLENGTH*Math.sin(Math.toRadians(ARM.getAngle())); // -90 ersetzen
					WHEELS.setWheelSpeed(necessaryWheelspeed);
				}while(ARM.isMoving); // isMoving ins Interface
				
//				isMoving
//
//				public boolean isMoving()
//				This method returns true if the motor is attempting to rotate. The return value may not correspond to the actual motor movement.
//				For example, If the motor is stalled, isMoving() will return true. 
//				After flt() is called, this method will return false even though the motor axle may continue to rotate by inertia. If the motor is stalled, isMoving() will return true. . A stall can be detected by calling isStalled();
//				Specified by:
//				isMoving in interface BaseMotor
//				Returns:
//				true iff the motor is attempting to rotate.
				waitForArm();
				waitForWheels();
	}
	
	/** Geschwindigkeit wird in Steps angepasst.
	 * Fuer jeden Step wird neue distanz fuer wheels berechnet
	 * geschwindigkeitsanpassung durch Funktion
	 * 
	 * Der Arm wird mit konstanter geschwindigkeit gestartet,
	 * 
	 * durch die Forschleife wird bei jedem durchlauf eine distanz in einer berechneten geschwindigkeit durchlaufen
	 * summe der distanzen ergibt die gesamt distanz
	 */
	public void movePenJulius2(int xTarget, int yTarget, int steps) throws OutOfWorkspaceException{
		double startAngle = ARM.getAngle();
		double endAngle = Calc.getAnglePen(ArmModule.ARMLENGTH, xTarget);
		double yNow;
		double yDeviance;
		double yDevianceAngle;
		double ySum;
		double yStep;
		double necessaryWheelspeed;
				
				yNow = WHEELS.getYCenter() + Calc.getYCenterToPen(ArmModule.ARMLENGTH, startAngle);
				yDeviance = yTarget - yNow;
				yDevianceAngle = Calc.getYCenterToPen(ArmModule.ARMLENGTH, startAngle) - Calc.getYCenterToPen(getArmLength(), endAngle);
				ySum = yDevianceAngle + yDeviance;
				yStep = ySum/steps;
				
				moveArmTo(endAngle, true);
				
				for(int i1 = 1; i1 == steps; i1++){
					necessaryWheelspeed = ARM.getRotationSpeed()*ArmModule.ARMLENGTH*Math.sin(Math.toRadians(ARM.getAngle())); // -90 ersetzen
					WHEELS.setWheelSpeed(necessaryWheelspeed);
					moveWheels(yStep, true);
					WHEELS.setWheelSpeed(necessaryWheelspeed);
					waitForWheels();
				}
				waitForArm();
	}
	

	/** Mit Steps. 
	 * Die Bewegung wird in Abschnitte unterteilt.
	 * Fuer jedes Bewegungsdreieck wird die y distanz und der winnkel berechnet
	 * berechneter winkel und y distanz werden nacheinander gefahren
	 * Geschwindigkeiten bleiben konstant
	 */
	public void movePenJulius3(int xTarget, int steps) throws OutOfWorkspaceException{
		double startAngle = ARM.getAngle();
		double endAngle = Calc.getAnglePen(ArmModule.ARMLENGTH, xTarget);
		double angleDifference = endAngle - startAngle;
		double stepAngle = angleDifference/steps;
		double sector = Math.sqrt(2)*ArmModule.ARMLENGTH*Math.sqrt(1-Math.cos(Math.toRadians(stepAngle))); // stepAngle = Degree? --   c^2 = 2a^2(1-cos(gamma)) 
		double gamma;
		double yDevianceAngle;
		double toAngle;
		
		for(int i1=1; i1 == steps; i1++){
			gamma = i1*stepAngle;
			yDevianceAngle = Math.sin(Math.toRadians(gamma))*sector;
			toAngle = (i1*stepAngle)+startAngle;
			moveArmTo(toAngle, false);
			moveWheels(yDevianceAngle, false);
		}
	}
	
	
	/** Mit Steps. Die Bewegung wird in Abschnitte unterteilt. Fuer jedes Bewegungsdreieck wird die y distanz und der winnkel berechnet
	 * berechneter winkel und y distanz wird gleichzeitig gefahren
	 * die benoetigte Geschwindigkeit wird fuer jeden durchlauf der for schleife neu berechnet
	 */
	public void movePenJulius4(int xTarget, int steps) throws OutOfWorkspaceException{
		double startAngle = ARM.getAngle();
		double endAngle = Calc.getAnglePen(ArmModule.ARMLENGTH, xTarget);
		double angleDifference = endAngle - startAngle;
		double stepAngle = angleDifference/steps;
		double sector = Math.sqrt(2)*ArmModule.ARMLENGTH*Math.sqrt(1-Math.cos(Math.toRadians(stepAngle))); // stepAngle = Degree? --   c^2 = 2a^2(1-cos(gamma)) 
		double gamma;
		double necessaryWheelspeed;
		double toAngle;
		double yDevianceAngle;
		
		for(int i1=1; i1 == steps; i1++){
			
			toAngle = (i1*stepAngle)+startAngle;
			gamma = i1*stepAngle;
			yDevianceAngle = Math.sin(Math.toRadians(gamma))*sector;
			necessaryWheelspeed = ARM.getRotationSpeed()*ArmModule.ARMLENGTH*Math.sin(Math.toRadians((i1-0.5)*stepAngle)); // -90 ersetzen
			WHEELS.setWheelSpeed(necessaryWheelspeed);
			moveArmTo(toAngle, true);
			moveWheels(yDevianceAngle, true);
			WHEELS.setWheelSpeed(necessaryWheelspeed);
			waitForWheels();
		}
			waitForArm();
	}
	
	
	
	
	// Julius Versuch Ende
	
	/////////////////
	/////  ARM
	/////////////////
	
	public int getArmLength(){
		return ArmModule.ARMLENGTH;
	}
	
	public int getArmMinAngle(){
		return ARM.getArmMinAngle();
	}
	
	public int getArmMaxAngle(){
		return ARM.getArmMaxAngle();
	}
	
	public double getArmAngle(){
		return ARM.getAngle();
	}
	
	public double getArmRotationSpeed(){
		return ARM.getRotationSpeed();
	}
	
	public void setArmSpeed(int speed) throws IndexOutOfBoundsException{
		ARM.setArmSpeed(speed);
	}
	
	public void moveArmTo(double targetAngle) throws OutOfWorkspaceException{
		ARM.moveArmTo(targetAngle);
	}
	
	public void moveArmTo(double targetAngle, boolean immediateReturn) throws OutOfWorkspaceException{
		ARM.moveArmTo(targetAngle, immediateReturn);
	}
	
	public void waitForArm(){
		ARM.waitForArm();
	}
	
	public void stopArm(){
		ARM.stopArm();
	}
	
	
	/////////////////
	/////  PEN
	/////////////////
	
	public void setPen(boolean down){
		PEN.setPen(down);
	}
	
	public void stopPen(){
		PEN.stopPen();
	}
	
	
	/////////////////
	/////  WHEELS
	/////////////////
	
	public int getMaxFeed(){
		return WHEELS.getYCenterMax();
	}
	
	public double getYCenter(){
		return WHEELS.getYCenter();
	}

	public void setWheelSpeed(double speed) throws IndexOutOfBoundsException{
		WHEELS.setWheelSpeed(speed);
	}
	
	public void moveWheels(double distance){
		try {
			WHEELS.moveWheels(distance);
		} catch (OutOfWorkspaceException e) {
			stopAllMotors();
			LCD.drawString("OoW in moveWheels()", 0, 8);
		}
	}
	
	public void moveWheels(double distance, boolean immediateReturn) throws OutOfWorkspaceException{
		try {
			WHEELS.moveWheels(distance, immediateReturn);
		} catch (OutOfWorkspaceException e) {
			stopAllMotors();
			LCD.drawString("OoW in moveWheels()", 0, 8);
		}
	}
	
	public void moveWheelsForward(){
		try {
			WHEELS.moveWheelsForward();
		} catch (OutOfWorkspaceException e) {
			stopAllMotors();
			LCD.drawString("OoW in moveWheelsForward()", 0, 8);
		}
	}
	
	public void moveWheelsBackward(){
		try {
			WHEELS.moveWheelsBackward();
		} catch (OutOfWorkspaceException e) {
			stopAllMotors();
			LCD.drawString("OoW in moveWheelsBackward()", 0, 8);
		}
	}
	
	public void waitForWheels(){
		WHEELS.waitForWheels();
	}
	
	public void stopWheels(){
		WHEELS.stopWheels();
	}
}
