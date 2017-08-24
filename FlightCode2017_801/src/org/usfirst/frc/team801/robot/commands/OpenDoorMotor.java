package org.usfirst.frc.team801.robot.commands;

import org.usfirst.frc.team801.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class OpenDoorMotor extends Command {

    public double threshold;

	public OpenDoorMotor() {
    	requires(Robot.doorOpener);

    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
        threshold = Robot.doorOpener.getMotorValues();
    	Robot.doorOpener.startUp();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return (threshold > 0.8);
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
