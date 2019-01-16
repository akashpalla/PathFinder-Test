/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team115.robot;

import org.usfirst.frc.team115.robot.commands.CycleCommand;
import org.usfirst.frc.team115.robot.commands.FollowProfile;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	Joystick driver;
	JoystickButton motionProfiling;
	JoystickButton cycle;

	public OI () {
		driver = new Joystick(0);
		motionProfiling = new JoystickButton(driver, 3);
		cycle = new JoystickButton(driver, 5);
		
		cycle.whenPressed(new CycleCommand());
		motionProfiling.whenPressed(new FollowProfile());
	}
	
	public boolean getCycle () {
		return cycle.get();
	}

}
