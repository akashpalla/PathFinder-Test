/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Waypoint;

public class VisionPath extends Command {
  public VisionPath() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double distance = Robot.drivetrain.getDistanceArea();
    double targetAngle = Robot.drivetrain.findNearestAngle();
    double angle2 = Robot.drivetrain.getAngle() - Robot.drivetrain.getGyroAngle();
    double xDistance = distance * Math.sin(Math.toRadians(angle2));
    double yDistance = distance * Math.cos(Math.toRadians(angle2));
    Waypoint[] points = new Waypoint[] {
      new Waypoint(0, 0, Robot.drivetrain.getGyroAngle()),
      new Waypoint(yDistance, xDistance, targetAngle),
    };
    new FollowProfile(points);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
