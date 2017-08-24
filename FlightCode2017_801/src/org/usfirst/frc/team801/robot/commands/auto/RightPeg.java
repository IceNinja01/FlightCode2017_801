package org.usfirst.frc.team801.robot.commands.auto;

import org.usfirst.frc.team801.robot.commands.chassis.DriveHoldHeadingCMD;
import org.usfirst.frc.team801.robot.commands.chassis.TurnWheelsToHeading;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightPeg extends CommandGroup {

    public RightPeg() {
//       	RobotMap.imu.reset();
//    	double angleCMD = Robot.chassis.getGyroAngle();
    	addSequential(new TurnWheelsToHeading(),0.5);
    	addSequential(new DriveHoldHeadingCMD(0.0, -0.5, 0.0),2.5);
//    	addSequential(new TurnToHeading(310.0),1.0);
////    	//
//        addSequential(new GearInMotor(),2.0);
//    	addSequential(new TurnWheelsToHeading(),2.0);
//    	addSequential(new TurnWheelsVisionAndDrive(-0.7, 20.0, 310.0),6);
    }
}
