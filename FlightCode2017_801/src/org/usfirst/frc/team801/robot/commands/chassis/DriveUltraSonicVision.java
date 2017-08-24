package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;
import org.usfirst.frc.team801.robot.Utilities.Utils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveUltraSonicVision extends Command {
	private double x,y,z,angleCmd_Deg;
    private double threshold;
	private double strafe;
	private double setPoint;
	private double setPointx;
	private double setPointz;
	private double setUltra;


    public DriveUltraSonicVision(double x1, double y1, double threshold){
    	requires(Robot.chassis);
//    	requires(Robot.camera);
    	this.threshold = threshold;
    	x = x1;
    	y = y1;
    	
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize(){
    	angleCmd_Deg = Robot.chassis.getGyroAngle();
    	
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	setUltra = Robot.chassis.getUltraDistance();
//    	setPoint = Robot.camera.getCenterX();
    	setPointx = Utils.limitMagnitude(Utils.joyExpo(setPoint/60,1.0), 0, 0.5);
    	setPointz = Utils.limitMagnitude(Utils.joyExpo(setPoint/60,0.1), 0, 0.2);
//    	Robot.chassis.turnToHeading(setPointx, 0.0, angleCmd_Deg , 0.0);

    	Robot.chassis.turnToHeading(0.1, y, angleCmd_Deg , angleCmd_Deg);
        
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
		return setUltra<=threshold;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.chassis.stop();
    	Robot.chassis.brakeOn();
    	Robot.chassis.brakeOff();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}
