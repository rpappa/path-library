package com.rpappa.paths;

import java.util.function.Function;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * Run a path, from a very theory-based, robot-agnostic approach Find error,
 * close error
 * @author Ryan Pappa
 */
public abstract class RunPath extends CommandGroup {
	protected Path path;

	protected Function<Double, Double> speed;

	public RunPath(Path path, double speed) {
		this.path = path;
		this.speed = x -> speed;
	}

	public RunPath(Path path, Function<Double, Double> speed) {
		this.path = path;
		this.speed = speed;
	}

	/**
	 * Get robot yaw
	 * 
	 * @return double yaw
	 */
	protected abstract double getYaw();

	/**
	 * Get the robot's distance
	 * 
	 * @return double distance, in inches
	 */
	public abstract double getDistance();
	
	/**
	 * Close the error, for example using a PID loop
	 * @param error the error from the path in degrees 
	 */
	protected abstract void closeError(double error);

	protected double deltaAngle(double currentAngle) {
		double currentSlope = Math.tan(currentAngle * Math.PI / 180);
		double nextSlope = dydx(getDistance());

		double angle = Math.atan(
				(nextSlope - currentSlope) / (1 + currentSlope * nextSlope))
				* 180 / Math.PI;

		return angle;
	}

	protected double dydx(double s) {
		PathSegment segment = path.getPathAtDistance(s);
		return segment.getDerivative().apply(
				(s - path.getTotalOfCompletedPaths(s)) / segment.getLength());
	}

	protected double speed() {
		return speed.apply(getDistance() / path.getTotalLength());
	}

	////////////////////////
	//    Command flow    //
	////////////////////////

	/**
	 * Do things like reset gyro, reset encoders etc
	 */
	protected abstract void initialize();

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double error = -deltaAngle(getYaw());
		
		closeError(error);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		try {
			// System.out.println(path.getPathAtDistance(Robot.drive.getRightDistance()).getLength());
			return Math.abs(getDistance()) > (path.getTotalLength());
		} catch (Exception e) {
			System.err.println("Unexpected error in RunPath.isFinished()");
			System.err.println(e);
			return true;
		}
	}

	/**
	 * Execute at the end. Probably want to stop the robot.
	 */
	protected abstract void end();

	/**
	 * Called if the command is interrupted. By default calls {@code end()}
	 */
	protected void interrupted() {
		end();
	}
}
