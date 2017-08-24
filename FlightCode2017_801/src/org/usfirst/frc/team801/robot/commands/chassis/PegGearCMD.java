package org.usfirst.frc.team801.robot.commands.chassis;

import org.usfirst.frc.team801.robot.Robot;
import org.usfirst.frc.team801.robot.commands.chassis.TurnWheelsToHeading;
import org.usfirst.frc.team801.robot.commands.chassis.TurnWheelsVisionAndDrive;
import org.usfirst.frc.team801.robot.commands.gear.GearInMotor;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PegGearCMD extends CommandGroup {

    public PegGearCMD() {
    	addSequential(new TurnWheelsToHeading(),0.5);
        addParallel(new TurnWheelsVisionAndDrive(-0.5, 10.0,Robot.chassis.getGyroAngle()));
        addSequential(new GearInMotor());


    }
}
