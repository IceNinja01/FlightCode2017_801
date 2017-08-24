package org.usfirst.frc.team801.robot.subsystems;

import org.usfirst.frc.team801.robot.RobotMap;
import org.usfirst.frc.team801.robot.commands.gather.GatherIdle;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Gather extends Subsystem {
	public static CANTalon gatMotor = RobotMap.gather;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public Gather(){
		
	}
    @Override
	public void initDefaultCommand() {
    	setDefaultCommand(new GatherIdle());

    }
   	public void setMotor(double motorValue){
    	gatMotor.set(motorValue);
    }
   	public void startUp(){
    	gatMotor.set(-1.0);
//    	Timer.delay(0.3);
//    	gatMotor.set(0.0);
    }
   	public void startDown(){
    	gatMotor.set(1.0);
//    	Timer.delay(0.3);
//    	gatMotor.set(0.0);
    }
	public void stop(){
   		gatMotor.set(0.0);
   	}
}

