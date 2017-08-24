package org.usfirst.frc.team801.robot.subsystems;

import org.usfirst.frc.team801.robot.RobotMap;
import org.usfirst.frc.team801.robot.commands.lift.LiftUp;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem {
	public static CANTalon liftMotor = RobotMap.lift;
	

	public Lift(){
		
	}
	
    @Override
	public void initDefaultCommand() {
    	setDefaultCommand(new LiftUp());
     
    }

	public void setMotor(double motorValue){
    	liftMotor.set(motorValue);
    }
}

