package org.usfirst.frc.team801.robot.commands.gather;

import org.usfirst.frc.team801.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GatherDown extends Command {

    public GatherDown() {
    	requires(Robot.gather);
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	Robot.gather.startDown();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.gather.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}
