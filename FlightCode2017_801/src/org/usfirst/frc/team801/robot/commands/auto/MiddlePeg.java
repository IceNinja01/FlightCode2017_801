package org.usfirst.frc.team801.robot.commands.auto;

import org.usfirst.frc.team801.robot.Robot;
import org.usfirst.frc.team801.robot.commands.chassis.DriveHoldHeadingCMD;
import org.usfirst.frc.team801.robot.commands.chassis.TurnWheelsToHeading;
import org.usfirst.frc.team801.robot.commands.chassis.TurnWheelsVisionAndDrive;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MiddlePeg extends CommandGroup {

    public double angleCMD3;

	public MiddlePeg() {
//    	RobotMap.imu.reset();
//    	Timer.delay(1.0);

    	addSequential(new TurnWheelsToHeading(),0.5);
//        addSequential(new GearInMotor(),3.0);
    	addSequential(new DriveHoldHeadingCMD(0.0, -0.3, Robot.chassis.getGyroAngle()),1.5);
        //
    	addSequential(new TurnWheelsVisionAndDrive(-0.3, 13.0, Robot.chassis.getGyroAngle()),6);
        
        //
//    	addSequential(new DriveHoldHeadingCMD(0.0, 0.2, 0.0),0.5);

//        addParallel(new BackDriveHoldHeadingUltraSound(0.0, 0.2, 0.0, 18.0),1.0);

    }
}
