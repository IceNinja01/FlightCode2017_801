package org.usfirst.frc.team801.robot.commands.gear;

import org.usfirst.frc.team801.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GearInMotor extends Command {

    public double[] threshold;

	public GearInMotor() {
    	requires(Robot.gearIn);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
        threshold = Robot.gearIn.getMotorValues();
    	Robot.gearIn.startUp();
    	SmartDashboard.putNumber("Motor1", threshold[0]);
    	SmartDashboard.putNumber("Motor2", threshold[1]);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return ((threshold[0] > 0.8) && (threshold[1] > 0.8));
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.gearIn.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}
