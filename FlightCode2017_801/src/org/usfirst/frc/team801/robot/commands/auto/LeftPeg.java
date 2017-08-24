package org.usfirst.frc.team801.robot.commands.auto;

import org.usfirst.frc.team801.robot.Robot;
import org.usfirst.frc.team801.robot.RobotMap;
import org.usfirst.frc.team801.robot.commands.chassis.DriveHoldHeadingCMD;
import org.usfirst.frc.team801.robot.commands.chassis.TurnToHeading;
import org.usfirst.frc.team801.robot.commands.chassis.TurnWheelsToHeading;
import org.usfirst.frc.team801.robot.commands.chassis.TurnWheelsVisionAndDrive;
import org.usfirst.frc.team801.robot.commands.gear.GearInMotor;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftPeg extends CommandGroup {

    public LeftPeg() {
       	RobotMap.imu.reset();
    	double angleCMD = Robot.chassis.getGyroAngle();
    	addSequential(new TurnWheelsToHeading(),0.5);
    	addSequential(new DriveHoldHeadingCMD(0.0, -0.5, angleCMD),2.5);
    	addSequential(new TurnToHeading(50.0),1.0);
    	//
        addSequential(new GearInMotor(),2.0);
    	addSequential(new TurnWheelsToHeading(),2.0);
    	angleCMD = Robot.chassis.getGyroAngle();
    	addSequential(new TurnWheelsVisionAndDrive(-0.7, 20.0, angleCMD),6);
    }
}
