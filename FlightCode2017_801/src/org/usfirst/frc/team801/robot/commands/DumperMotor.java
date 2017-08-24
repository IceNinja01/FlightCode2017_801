package org.usfirst.frc.team801.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team801.robot.Robot;


public class DumperMotor extends Command {
	
	public DumperMotor(){
		requires(Robot.dumper);
	}
	@Override
	protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	Robot.dumper.startUp();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.dumper.stop();
    }
    
    @Override
	protected void interrupted() {
    	end();
    }


}
