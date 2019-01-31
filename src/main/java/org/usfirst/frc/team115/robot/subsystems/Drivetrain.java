package org.usfirst.frc.team115.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.commands.DriveWithJoystick;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;

public class Drivetrain extends Subsystem{
	
	public CANSparkMax frontLeft, backLeft, frontRight, backRight;
	public CANEncoder left, right, left2, right2;
	public EncoderFollower leftFollower;
	public EncoderFollower rightFollower;
	public AHRS navX;
	NetworkTable table; 
	NetworkTableEntry tx; 
	NetworkTableEntry ty; 
	NetworkTableEntry ta;
	//SpeedControllerGroup leftDrive;
  	//SpeedControllerGroup rightDrive;
  	//DifferentialDrive drive;

	public Drivetrain() {
		frontLeft = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
		frontRight = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
		backLeft = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
		backRight = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
		left = new CANEncoder(frontLeft);
		right = new CANEncoder(frontRight);
		left2 = new CANEncoder(backLeft);
		right2 = new CANEncoder(backRight);	
		leftFollower = new EncoderFollower();
		rightFollower = new EncoderFollower();

		//leftDrive = new SpeedControllerGroup(frontLeft, backLeft);
    	//rightDrive = new SpeedControllerGroup(frontRight, backRight);
    	//drive = new DifferentialDrive(leftDrive, rightDrive);
		
		frontLeft.setIdleMode(IdleMode.kBrake);
		backLeft.setIdleMode(IdleMode.kBrake);
		frontRight.setIdleMode(IdleMode.kBrake);
		backRight.setIdleMode(IdleMode.kBrake);

		leftFollower.configurePIDVA(0, 0, 0, 1/ Constants.MAX_VELOCITY + 0.9, 0);
		rightFollower.configurePIDVA(0, 0, 0, 1/ Constants.MAX_VELOCITY, 0);
		navX = new AHRS(SPI.Port.kMXP);


		table = NetworkTableInstance.getDefault().getTable("limelight");
		tx = table.getEntry("tx");
	 	ty = table.getEntry("ty");
		ta = table.getEntry("ta");		
	}
	
	
	public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
		frontLeft.set(leftOutput);
		backLeft.set(leftOutput);
		frontRight.set(rightOutput);
		backRight.set(rightOutput);

	}


	public double getAngle() {
		return tx.getDouble(0.0);
	}

	public double getDistance() {
		double angle = ty.getDouble(0);
		return (Constants.TARGET_HEIGHT - Constants.CAMERA_HEIGHT)/Math.tan(Math.toRadians(Constants.CAMERA_ANGLE + angle));
	}

	public double getDistanceArea(){
		double area = ta.getDouble(0);

		return Math.log10(area/5.45)/Math.log10(0.5) + 2;
	}

	public void updateMotionFollowing() { 
		double leftOutput = leftFollower.calculate((int)(left.getPosition() *42));
		double rightOutput = rightFollower.calculate((int)(right.getPosition() *42));
			
		double gyro = navX.getAngle();
		double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftFollower.getHeading()));
		SmartDashboard.putNumber("DESIRED HEADING", desiredHeading);
		double angleDifference = -(desiredHeading - gyro);
		double turn  = 0.8 * (-1.0/80.0) * angleDifference;
		
		SmartDashboard.putNumber("ANGLE ERROR", desiredHeading - gyro);
		leftOutput +=turn;

		rightOutput -= turn;
		

		SmartDashboard.putNumber("LEFT OUTPUT:", leftOutput);
		SmartDashboard.putNumber("RIGHT OUTPUT:", rightOutput);

		setLeftRightMotorOutputs(leftOutput, -rightOutput);
	}

	public void drive(double throttle, double wheel, boolean isQuickTurn) {
		//drive.curvatureDrive(throttle, wheel, isQuickTurn);
	}

	public double findNearestAngle(){
		double gyroAngle = getGyroAngle();
		int num = (int)(getGyroAngle())/90;
		int high = 90 * (num+1);
		int low = 90 * num;
		int target;

		if(gyroAngle < 0)
			high = 90 * num;
			low = 90 * (num - 1);

		if(Math.abs(high-gyroAngle) < Math.abs(low - gyroAngle))
			target = high;
		else
			target = low;


		return target;
	}

	public double getGyroAngle() {
		return navX.getAngle();
	}

	protected void initDefaultCommand() {
		//setDefaultCommand(new DriveWithJoystick());
	}


	
	
	
}
