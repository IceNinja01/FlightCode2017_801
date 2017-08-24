package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;
import org.usfirst.frc.team801.robot.Utilities.Utils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToHeading extends Command {
	private double angleCmd_Deg;
	
    public TurnToHeading(double angleCmd_Deg) {
    	requires(Robot.chassis);
    	this.angleCmd_Deg = angleCmd_Deg;
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	angleCmd_Deg -= Robot.angleCmd_DegCon;
    	angleCmd_Deg = Utils.wrapAngle0To360Deg(angleCmd_Deg);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {

    	Robot.chassis.turnToHeading(0.0, 0.0, angleCmd_Deg, 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return Robot.chassis.onTarget();
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
