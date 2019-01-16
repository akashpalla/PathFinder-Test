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

	public FollowProfile() {
		Waypoint[] points = new Waypoint[] {
					new Waypoint(0, 0, 0),
					new Waypoint(2, 0, 0),
					new Waypoint(4, 0, 0),
		};
		scheduler = Executors.newScheduledThreadPool(1);
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, Constants.TIME_STEP, Constants.MAX_VELOCITY, Constants.MAX_ACCEL, Constants.MAX_JERK);
		trajectory = Pathfinder.generate(points, config);
		counter=0;	
		modifier = new TankModifier(trajectory).modify(Constants.WHEELBASE_WIDTH);
	
	}
	
	protected void initialize() {
		SmartDashboard.putNumber("START", 0.0);
		motionFollower = scheduler.scheduleAtFixedRate(new Runnable() {
			public void run() {
				SmartDashboard.putNumber("INIT", counter);
				Robot.drivetrain.updateMotionFollowing();
				counter++;
			}
			
		}, (int)(Constants.TIME_STEP*1000), (int)(Constants.TIME_STEP*1000), TimeUnit.MILLISECONDS);
		
		Robot.drivetrain.leftFollower.setTrajectory(modifier.getLeftTrajectory());
		Robot.drivetrain.rightFollower.setTrajectory(modifier.getRightTrajectory());
		Robot.drivetrain.leftFollower.configureEncoder((int)Robot.drivetrain.left.getPosition(), 1000, Constants.WHEEL_DIAMETER);
		Robot.drivetrain.rightFollower.configureEncoder((int)Robot.drivetrain.right.getPosition(), 1000, Constants.WHEEL_DIAMETER);

	}
	
	protected void execute() { }


	protected boolean isFinished() {
		return Robot.drivetrain.leftFollower.isFinished() && Robot.drivetrain.rightFollower.isFinished();
	}
	
	
	protected void end() {
		motionFollower.cancel(true);  
		Robot.drivetrain.setLeftRightMotorOutputs(0, 0);
		System.out.print("Finished");
	}
}
