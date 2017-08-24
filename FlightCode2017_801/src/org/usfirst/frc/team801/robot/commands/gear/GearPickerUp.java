package org.usfirst.frc.team801.robot.commands.gear;

import org.usfirst.frc.team801.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GearPickerUp extends Command {

    private int counter;
	private int setPoint;

	public GearPickerUp(int setPoint) {
        requires(Robot.gearPicker);
        this.setPoint = setPoint;
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	counter = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	counter = Robot.gearPicker.getEncoder(counter);
    	Robot.gearPicker.startUp();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return counter>=setPoint;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.gearPicker.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}
