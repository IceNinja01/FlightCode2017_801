package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnWheelsAndDrive extends Command {

	private double angleCmd_Deg;
	private double speed;
	private double setPoint;
	private double setPointx;
    public TurnWheelsAndDrive(double angleCmd_Deg, double speed) {
         requires(Robot.chassis);
         this.speed = speed;
         this.angleCmd_Deg = angleCmd_Deg;
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	
    	Robot.chassis.pointWheelsDrive(angleCmd_Deg, speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return Robot.chassis.getUltraDistance()<=setPoint;
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
