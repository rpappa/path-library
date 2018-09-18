package com.rpappa.paths;

import java.util.function.Function;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * RunPath for a tank drive robot
 * @author Ryan Pappa
 */
public abstract class RunPathTankDrive extends RunPath {
	protected double arcDivisor = 15;

	private double leftSpeed = 0;
	private double rightSpeed = 0;
	
	private Path path;
	
	private Function<Double, Double> speed;
	
   public RunPathTankDrive(Path path, double speed) {
    	super(path, speed);
    	this.leftSpeed = -speed;
    	this.rightSpeed = -speed;
    }
    
    public RunPathTankDrive(Path path, Function<Double, Double> speed) {
    	super(path, speed);
    	this.leftSpeed = speed.apply(0.0);
    	this.rightSpeed = speed.apply(0.0);
    }
    
    protected abstract void setDriveRails(double leftSpeed, double rightSpeed);
    
    protected abstract void resetBothEncoders();
    
    protected abstract void resetGyro();
    
    protected abstract double getYaw();
    
    public abstract double getDistance();
    
    @Override
    protected void closeError(double error) {
    	leftSpeed = speed();
    	rightSpeed = speed();
    	
//    	System.out.println("error: " + error);
    	if(Math.abs(getDistance()) > 3) {
    		double speed = leftSpeed;
        	setDriveRails(
        			(leftSpeed+((error)/(arcDivisor/Math.abs(speed)))), 
        			(rightSpeed-(((error)/(arcDivisor/Math.abs(speed))))));
        	
    	} else {
        	setDriveRails(leftSpeed, rightSpeed);
    	}
    }

    // Called just before this Command runs the first time
    protected void initialize() {	
    	setDriveRails(leftSpeed, rightSpeed);
    	resetBothEncoders();
    	resetGyro();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	setDriveRails(0, 0);
    }
}
