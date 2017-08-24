package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnWheelsToHeading extends Command {

	private double angleCmd_Deg;
    public TurnWheelsToHeading() {
         requires(Robot.chassis);
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	angleCmd_Deg = Robot.chassis.getGyroAngle();

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	Robot.chassis.pointWheels(angleCmd_Deg);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.chassis.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}
