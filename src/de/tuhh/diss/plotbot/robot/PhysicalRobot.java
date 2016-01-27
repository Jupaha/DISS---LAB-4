package de.tuhh.diss.plotbot.robot;

import de.tuhh.diss.plotbot.exceptions.MotorException;
import de.tuhh.diss.plotbot.exceptions.OutOfWorkspaceException;
import de.tuhh.diss.plotbot.utilities.Calc;
import lejos.nxt.Button;
import lejos.nxt.LCD;

public class PhysicalRobot implements RobotInterface{

	///////////////////////////////////////////////////////
	//	VARIABLES
	///////////////////////////////////////////////////////
	public static final PhysicalRobot ROBOT = new PhysicalRobot(); //TODO: unstatic
	
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
	public void movePenJulius1(double xStart, double xlength, int steps) throws OutOfWorkspaceException{
		double xTarget = xStart+xlength;
		double startAngle = ARM.getAngle();
		double endAngle = Calc.getAnglePen(ArmModule.ARMLENGTH, xTarget);
		double yDeviance = Calc.getYCenterToPen(ArmModule.ARMLENGTH, startAngle) - Calc.getYCenterToPen(getArmLength(), endAngle);;
		double yStep = yDeviance/steps;
		double necessaryWheelspeed;
		double angleDifference = endAngle - startAngle;
		double stepAngle = angleDifference/steps;
		double toAngle;
		double previousAngle;
		double armSpeed = ARM.getMotorSpeed()/ARM.ARMGEARRATIO;
		int i=1;
				while(i < steps+1){
				LCD.clear();
	
				toAngle = (i*stepAngle)+startAngle;	
				previousAngle = startAngle+(i-1)*stepAngle;
				necessaryWheelspeed = armSpeed*Math.sin(Math.toRadians(((stepAngle/2)+previousAngle)));
	
				WHEELS.setWheelSpeed(necessaryWheelspeed);
				
				moveWheels(yStep, true);
				moveArmTo(toAngle, true);
				
				waitForArm();
				waitForWheels();
				
				i++;
				}
	}
	
	/** Mit Steps. Fuer jeden Step Geschwindigkeitsanpassung durch Funktion
	 * 
	 * Der Arm wird mit konstanter geschwindigkeit gestartet,
	 * 
	 * durch die Forschleife wird bei jedem durchlauf eine distanz in einer berechneten geschwindigkeit durchlaufen
	 * summe der distanzen ergibt die gesamt distanz
	 */
	public void movePenJulius2(double xStart, double xlength, int steps) throws OutOfWorkspaceException{
		double xTarget = xStart+xlength;
		double startAngle = ARM.getAngle();
		double endAngle = Calc.getAnglePen(ArmModule.ARMLENGTH, xTarget);
		double yNow;
		double yDeviance;
		double yDevianceAngle;
		double ySum;
		double yStep;
		double necessaryWheelspeed;	
		
				yDevianceAngle = Calc.getYCenterToPen(ArmModule.ARMLENGTH, startAngle) - Calc.getYCenterToPen(getArmLength(), endAngle);
				ySum = yDevianceAngle;
				yStep = ySum/steps;
				
				moveArmTo(endAngle, true);
				int i = 1;
				while(i < steps+1){
					necessaryWheelspeed = ARM.getMotorSpeed()*ArmModule.ARMLENGTH*Math.sin(Math.toRadians(ARM.getAngle())); // -90 ersetzen
					LCD.drawString("NSPD: " + String.valueOf(necessaryWheelspeed), 0, 6);
					WHEELS.setWheelSpeed(necessaryWheelspeed);
					moveWheels(yStep, true);
					WHEELS.setWheelSpeed(necessaryWheelspeed);
					waitForWheels();
					i++;
				}
				waitForArm();
	}
	

	/** Mit Steps. 
	 * Die Bewegung wird in Abschnitte unterteilt.
	 * Fuer jedes Bewegungsdreieck wird die y distanz und der winnkel berechnet
	 * berechneter winkel und y distanz werden nacheinander gefahren
	 * Geschwindigkeiten bleiben konstant
	 */
	public void movePenJulius3(double xStart, double xlength, int steps) throws OutOfWorkspaceException{
		double xTarget = xStart+xlength;
		double startAngle = ARM.getAngle();
		double endAngle = Calc.getAnglePen(ArmModule.ARMLENGTH, xTarget);
		double angleDifference = endAngle - startAngle;
		double stepAngle = angleDifference/steps;
		double sector = Math.sqrt(2)*ArmModule.ARMLENGTH*Math.sqrt(1-Math.cos(Math.toRadians(stepAngle))); // stepAngle = Degree? --   c^2 = 2a^2(1-cos(gamma)) 
		double gamma;
		double yDevianceAngle;
		double toAngle;
		
		int i = 1;
		while(i < steps+1){
			gamma = i*stepAngle;
			yDevianceAngle = Math.sin(Math.toRadians(gamma))*sector;
			toAngle = (i*stepAngle)+startAngle;
			moveArmTo(toAngle, false);
			moveWheels(yDevianceAngle, false);
			i++;
		}
	}
	
	
	/** Mit Steps. Die Bewegung wird in Abschnitte unterteilt. Fuer jedes Bewegungsdreieck wird die y distanz und der winnkel berechnet
	 * berechneter winkel und y distanz wird gleichzeitig gefahren
	 * die benoetigte Geschwindigkeit wird fuer jeden durchlauf der for schleife neu berechnet
	 */
	public void movePenJulius4(double xStart, double xlength, int steps) throws OutOfWorkspaceException{
		double xTarget = xStart+xlength;
		double startAngle = ARM.getAngle();
		double endAngle = Calc.getAnglePen(ArmModule.ARMLENGTH, xTarget);
		double angleDifference = endAngle - startAngle;
		double stepAngle = angleDifference/steps;
		double sector = Math.sqrt(2)*ArmModule.ARMLENGTH*Math.sqrt(1-Math.cos(Math.toRadians(stepAngle))); // stepAngle = Degree? --   c^2 = 2a^2(1-cos(gamma)) 
		double gamma;
		double necessaryWheelspeed;
		double toAngle;
		double yDevianceAngle;
		
		int i = 1;
		while(i < steps+1){
			
			toAngle = (i*stepAngle)+startAngle;
			gamma = i*stepAngle;
			yDevianceAngle = Math.sin(Math.toRadians(gamma))*sector;
			if(ARM.getRotationSpeed()==0){
				moveArmTo(toAngle, true);
			}else{
			necessaryWheelspeed = ARM.getMotorSpeed()*ArmModule.ARMLENGTH*Math.sin(Math.toRadians((i-0.5)*stepAngle)); // -90 ersetzen
			LCD.drawString("NSPD: " + String.valueOf(necessaryWheelspeed), 0, 6);
			WHEELS.setWheelSpeed(necessaryWheelspeed);
			moveArmTo(toAngle, true);
			moveWheels(yDevianceAngle, true);
			WHEELS.setWheelSpeed(necessaryWheelspeed);
			waitForWheels();
			};
			i++;
		}
			waitForArm();
	}
	
	
	
	
	// Julius Versuch Ende
	

	public void movePenToLennart(double xTarget, double yTarget) throws MotorException{
		
		try{
			double targetAngle = Calc.getAnglePen(ArmModule.getArmLength(), xTarget);
			double yOfTargetAngle = Calc.getYCenterToPen(ArmModule.getArmLength(), targetAngle);
			double currentYPos = this.getYCenter();
			double distance = yTarget - yOfTargetAngle - currentYPos;
			
			this.moveArmTo(targetAngle, true);
			this.moveWheels(distance, true);
			
			this.waitForArm();
			this.waitForWheels();
		} catch(OutOfWorkspaceException e){
			
			this.stopAllMotors();
			throw new MotorException();
		}
	}
	
	public void movePenVerticalLennart(double xStart, double yStart, double length) throws MotorException, OutOfWorkspaceException{
		
		this.movePenToLennart(xStart, yStart);
		moveWheels(length);
	}
	
	public void movePenHorizontalLennart(double xStart, double yStart, double length, int amountOfSteps) throws MotorException{
		
		try{
			int armLength = ArmModule.getArmLength();
			int tolerance = 5;			//TODO Konstante machen
			double angleStepTarget;
			boolean targetReached = false;
			
			amountOfSteps = Math.abs(amountOfSteps);
			
			movePenToLennart(xStart, yStart);
			
			do{
				double angleCurrent = this.getArmAngle();
				double yCurrent = this.getYCenter() + Calc.getYCenterToPen(armLength, angleCurrent);
				double angleTarget = Calc.getAnglePen(armLength, xStart + length);
				
				double yDelta = Calc.getYCenterToPen(armLength, angleTarget) - Calc.getYCenterToPen(armLength, angleCurrent);	
				double yStep = yDelta / amountOfSteps;
				
				for(int i = 1; i == amountOfSteps; i++){
					
					angleStepTarget = Calc.getAnglePenOfY(armLength, yCurrent + i * yStep); 
					this.moveArmTo(angleStepTarget, true);
					this.moveWheels(yStep, true);
					
					this.waitForWheels();
					this.waitForArm();
				}
	
				//TODO Methode zu double aendern
				double armAngle = this.getArmAngle();
				targetReached = Calc.targetReachedSufficently((int) Calc.getXPositionPen(armLength, armAngle),(int) (this.getYCenter() + Calc.getYCenterToPen(armLength, armAngle)), (int) (xStart  + length), (int) yStart, tolerance);
				
				amountOfSteps = 1;		// TODO evtl aendern
			} while(!targetReached);
		} catch(OutOfWorkspaceException e){
			
			this.stopAllMotors();
			throw new MotorException();
		}
	}
	
	public void movePenHorizontalLennart2(double xStart, double yStart, double length, int yStep) throws MotorException{
		
		try{
			movePenToLennart(xStart, yStart);
			
			int armLength = ArmModule.getArmLength();
			
			double angleStart = Calc.getAnglePen(armLength, xStart);
			double angleTarget = Calc.getAnglePen(armLength, xStart + length);
			
			double yDelta = Calc.getYCenterToPen(armLength, angleTarget) - Calc.getYCenterToPen(armLength, angleStart);
			
			int amountOfSteps = Math.abs((int) (yDelta / yStep));
			
			yStep = Math.abs(yStep);
			if(yDelta < 0){
				yStep = -yStep;
			}
			
			double angleStepTarget;
			
			for(int i = 1; i == amountOfSteps; i++){
					
				angleStepTarget = Calc.getAnglePenOfY(armLength, yStart + i * yStep); 
				this.moveArmTo(angleStepTarget, true);
				this.moveWheels(yStep, true);
					
				this.waitForWheels();
				this.waitForArm();
			}
			
			double yRest = Calc.getYCenterToPen(armLength, angleTarget) - Calc.getYCenterToPen(armLength, this.getArmAngle());
			this.moveArmTo(angleTarget, true);
			this.moveWheels(yRest, true);
			
			this.waitForWheels();
			this.waitForArm();
			
		} catch(OutOfWorkspaceException e){
			
			this.stopAllMotors();
			throw new MotorException();
		}
	}

	
	/////////////////
	/////  ARM
	/////////////////
	
	public int getArmLength(){
		return ArmModule.ARMLENGTH;
	}
	
	public double getArmMinAngle(){
		return ARM.getArmMinAngle();
	}
	
	public double getArmMaxAngle(){
		return ARM.getArmMaxAngle();
	}
	
	public double getArmAngle(){
		return ARM.getAngle();
	}
	
	public int getArmMotorSpeed(){
		return ARM.getMotorSpeed();
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
	
	public void moveWheels(double distance) throws OutOfWorkspaceException{
		WHEELS.moveWheels(distance);
	}
	
	public void moveWheels(double distance, boolean immediateReturn) throws OutOfWorkspaceException{
		WHEELS.moveWheels(distance, immediateReturn);
	}
	
	public void moveWheelsForward() throws OutOfWorkspaceException{
		WHEELS.moveWheelsForward();
	}
	
	public void moveWheelsBackward() throws OutOfWorkspaceException{
		WHEELS.moveWheelsBackward();
	}
	
	public void waitForWheels(){
		WHEELS.waitForWheels();
	}
	
	public void stopWheels(){
		WHEELS.stopWheels();
	}
}
