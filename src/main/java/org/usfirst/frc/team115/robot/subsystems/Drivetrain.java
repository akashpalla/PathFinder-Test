package org.usfirst.frc.team115.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.followers.EncoderFollower;

public class Drivetrain extends Subsystem{
	
	public CANSparkMax frontLeft, backLeft, frontRight, backRight;
	public CANEncoder left, right;
	public EncoderFollower leftFollower;
	public EncoderFollower rightFollower;
	
	public Drivetrain() {
		frontLeft = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
		frontRight = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
		backLeft = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
		backRight = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
		left = new CANEncoder(frontLeft);
		right = new CANEncoder(frontRight);	
		leftFollower = new EncoderFollower();
		rightFollower = new EncoderFollower();
		leftFollower.configurePIDVA(1, 0, 0, 1, 0);
		rightFollower.configurePIDVA(1, 0, 0, 1, 0);
		
	}
	
	
	public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
		frontLeft.set(leftOutput);
		backLeft.set(leftOutput);
		frontRight.set(rightOutput);
		backRight.set( rightOutput);

	}

	public void updateMotionFollowing() {
		double leftOutput = leftFollower.calculate((int)left.getPosition());
		double rightOutput = rightFollower.calculate((int)left.getPosition());
		SmartDashboard.putNumber("LEFT OUTPUT:", leftOutput);
		SmartDashboard.putNumber("RIGHT OUTPUT:", rightOutput);

		setLeftRightMotorOutputs(leftOutput, rightOutput);
	}


	protected void initDefaultCommand() {
		
	}


	
	
	
}
