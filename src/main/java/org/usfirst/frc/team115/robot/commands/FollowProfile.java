package org.usfirst.frc.team115.robot.commands;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.Executors;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class FollowProfile extends Command {
	
	ScheduledExecutorService scheduler;
	ScheduledFuture<?> motionFollower;
	TankModifier modifier;
	public Trajectory trajectory;
	public int counter;

	public FollowProfile(Waypoint[] points) {
		scheduler = Executors.newScheduledThreadPool(1);
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, Constants.TIME_STEP, Constants.MAX_VELOCITY, Constants.MAX_ACCEL, Constants.MAX_JERK);
		trajectory = Pathfinder.generate(points, config);
		counter=0;	
		modifier = new TankModifier(trajectory).modify(Constants.WHEELBASE_WIDTH);
	
	}
	
	protected void initialize() {
		motionFollower = scheduler.scheduleAtFixedRate(new Runnable() {
			public void run() {
				Robot.drivetrain.updateMotionFollowing();
			}
			
		}, (int)(Constants.TIME_STEP*1000), (int)(Constants.TIME_STEP*1000), TimeUnit.MILLISECONDS);
		
		Robot.drivetrain.leftFollower.setTrajectory(modifier.getLeftTrajectory());
		Robot.drivetrain.rightFollower.setTrajectory(modifier.getRightTrajectory());
		Robot.drivetrain.leftFollower.configureEncoder((int)(Robot.drivetrain.left.getPosition() * 42) , Constants.TICKS_PER_ROTATION, Constants.WHEEL_DIAMETER);
		Robot.drivetrain.rightFollower.configureEncoder(-1 *(int)(Robot.drivetrain.right.getPosition()*42), Constants.TICKS_PER_ROTATION, Constants.WHEEL_DIAMETER);
		 
	}
	
	protected void execute() {

		Trajectory.Segment seg = trajectory.get(trajectory.length() - 1);
		double position = seg.position;
		double currDistance = (Robot.drivetrain.left.getPosition() /8.8) * Math.PI * Constants.WHEEL_DIAMETER;

		SmartDashboard.putNumber("CURRENT Distance (Meters)", currDistance);
		SmartDashboard.putNumber("ERROR", position - currDistance);
	 }


	protected boolean isFinished() {
	/*	Trajectory.Segment seg = trajectory.get(trajectory.length() - 1);
		double position = seg.position;
		double [] posArr = new double [10];

		double next = Robot.drivetrain.right.getPosition();
		double temp = 0;
		for(int i = 9; i <0; i--) {
			temp = posArr[i];
			posArr[i] = next;
			next = temp; 
		}

		if(Robot.drivetrain.leftFollower.isFinished() && Robot.drivetrain.rightFollower.isFinished()) {
			 for(int i = 0; i < 10; i++) {
					 if(Math.abs(Robot.drivetrain.left2.getPosition() - position) > Constants.MAX_OFFSET)
					 	return false;
			}
			return true; 	 
		}
		
		return false;
		*/
		return (Robot.drivetrain.leftFollower.isFinished() && Robot.drivetrain.rightFollower.isFinished());
	}
	
	
	protected void end() {
		motionFollower.cancel(true);  
		Robot.drivetrain.setLeftRightMotorOutputs(0, 0);
		System.out.print("Finished");
	}
}
