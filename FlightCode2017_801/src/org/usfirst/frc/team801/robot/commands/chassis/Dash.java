package org.usfirst.frc.team801.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Dash extends CommandGroup {

    public Dash() {
        // Add Commands here:
    	
        addSequential(new DriveHoldHeadingCMD(0.0, -0.3, 0),3);
        
    }
}
