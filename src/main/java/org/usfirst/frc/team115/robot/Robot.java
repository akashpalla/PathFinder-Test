/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team115.robot;


import com.revrobotics.CANSparkMax.IdleMode;

import org.usfirst.frc.team115.robot.commands.FollowProfile;
import org.usfirst.frc.team115.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static Drivetrain drivetrain;
	public static OI oi;
	public FollowProfile fp; 
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	public static int count;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();
		oi = new OI();
	//	count = fp.trajectory.length() -1;	
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		drivetrain.frontLeft.setIdleMode(IdleMode.kCoast);
		drivetrain.backLeft.setIdleMode(IdleMode.kCoast);
		drivetrain.frontRight.setIdleMode(IdleMode.kCoast);
		drivetrain.backRight.setIdleMode(IdleMode.kCoast);

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();

		double currDistance = (Robot.drivetrain.left.getPosition() /8.8) * Math.PI * Constants.WHEEL_DIAMETER;
		
		SmartDashboard.putNumber("CURRENT Distance (Meters) 2", currDistance);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {

		drivetrain.frontLeft.setIdleMode(IdleMode.kCoast);
		drivetrain.backLeft.setIdleMode(IdleMode.kCoast);
		drivetrain.frontRight.setIdleMode(IdleMode.kCoast);
		drivetrain.backRight.setIdleMode(IdleMode.kCoast);
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		drivetrain.navX.zeroYaw();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
	
		SmartDashboard.putNumber("NavX Angle:", this.drivetrain.navX.getAngle());
		SmartDashboard.putNumber("Left MotorOutput",  drivetrain.backLeft.get());
		SmartDashboard.putNumber("Right MotorOutput:", drivetrain.backRight.get());
		SmartDashboard.putNumber("Left Encoder Value:",drivetrain.left.getPosition());
		SmartDashboard.putNumber("Right Encoder Value", -1 *drivetrain.right.getPosition());
		SmartDashboard.putNumber("Current Distance (Trig):", drivetrain.getDistance());
		SmartDashboard.putNumber("Distance (Area)", drivetrain.getDistanceArea());
		SmartDashboard.putNumber("Nearest Angle", drivetrain.findNearestAngle()); 

		double distance = Robot.drivetrain.getDistance();
		double targetAngle = Robot.drivetrain.findNearestAngle();
		double angle2 = Robot.drivetrain.getAngle() + Robot.drivetrain.getGyroAngle();
		double xDistance = distance * Math.sin(Math.toRadians(angle2));
		double yDistance = distance * Math.cos(Math.toRadians(angle2));

		SmartDashboard.putNumber("YDISTANCE", yDistance);
		SmartDashboard.putNumber("XDistance", xDistance);
	}
	

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
