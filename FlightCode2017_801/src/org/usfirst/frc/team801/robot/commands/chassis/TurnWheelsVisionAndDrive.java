package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;
import org.usfirst.frc.team801.robot.Utilities.Utils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnWheelsVisionAndDrive extends Command {

	private double angleCmd_Deg;
	private double speed;
	private double setPoint;
	private double setPointx;
	private double angleInput;
    public TurnWheelsVisionAndDrive(double speed, double setPoint, double angleInput) {
         requires(Robot.chassis);
//         requires(Robot.camera);
         this.speed = speed;
         this.setPoint = setPoint;
         this.angleInput = angleInput;
         
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
//    	if(!Robot.camera.getProcessing()){//ensure that vision processing is on
//    		Robot.camera.enableProcessing();
//    	}
    	this.angleCmd_Deg =Robot.angleCmd_DegCon;
    	SmartDashboard.putNumber("turnGyro", angleCmd_Deg);
    	angleCmd_Deg -= angleInput;
    	angleCmd_Deg = Utils.wrapAngle0To360Deg(angleCmd_Deg);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
//    	setPointx = Robot.camera.getCenterX()*0.214; //degree/pixel => 68.5 / ImgWidth
    	
    	Robot.chassis.motorDriveHoldHeading(0.0, speed, angleCmd_Deg);
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
//    	Robot.camera.enableProcessing();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}
