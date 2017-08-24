package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class Fast extends Command
{
    public Fast()
    {
    	requires(Robot.chassis);
    }

    @Override
	protected void initialize()
    {
    	Robot.chassis.toggleFastMode();
    }

    @Override
	protected void execute()
    {
    }

    @Override
	protected boolean isFinished()
    {
    	return true;
    }

    @Override
	protected void end()
    {
    }
    
    @Override
	protected void interrupted()
    {
    	end();
    }
}
