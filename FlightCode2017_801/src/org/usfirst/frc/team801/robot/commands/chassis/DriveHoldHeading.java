package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;
import org.usfirst.frc.team801.robot.Utilities.Utils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveHoldHeading extends Command {
//This commands allows you to give angles and speed values to drive towards
	// Use this command with a timer in the addSequential(new Command(),timer(s))
	
	private double angleCmd;
	private double speed;
	private double gyroAngle;
	public double theta;
	private double x;
	private double y;
	private double z;

	public DriveHoldHeading(double angleCmd, double speed, double gyroAngle){
		requires(Robot.chassis);
		this.angleCmd = angleCmd;
		this.speed = speed;
		this.gyroAngle = gyroAngle;
	}
	
	 // Called just before this Command runs the first time
    @Override
	protected void initialize(){
    	//get x and y values from the angleCmd and speed
     	 theta+=90.0;
    	 theta = angleCmd * Math.PI / 180;//convert to radians
    	 x = speed * Math.cos(theta);
    	 y = speed * Math.sin(theta);
    	SmartDashboard.putNumber("theta", theta);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	
    	Robot.chassis.pointWheelsDrive(Utils.wrapAngle0To360Deg(angleCmd), speed);

        
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
		return false;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}


