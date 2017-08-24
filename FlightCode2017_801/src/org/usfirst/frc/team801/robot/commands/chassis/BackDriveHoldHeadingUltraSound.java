package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class BackDriveHoldHeadingUltraSound extends Command {
//This commands allows you to give angles and speed values to drive towards
	// Use this command with a timer in the addSequential(new Command(),timer(s))
	
	private double angleCmd;
	private double speed;
	private double gyroAngle;
	public double theta;
	private double x;
	private double y;
	private double z;
	private double setPoint;

	public BackDriveHoldHeadingUltraSound(double x, double y, double angleCmd,double setPoint){
		requires(Robot.chassis);
		this.angleCmd = angleCmd;
		this.x = x;
		this.y = y;
		this.setPoint = setPoint;
	}
	
	 // Called just before this Command runs the first time
    @Override
	protected void initialize(){
    	//get x and y values from the angleCmd and speed
     	 
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	Robot.chassis.motorDriveHoldHeading(x, y, angleCmd);

        
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
    	 return Robot.chassis.getUltraDistance()>=setPoint;
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


