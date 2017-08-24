package org.usfirst.frc.team801.robot.subsystems;


import org.usfirst.frc.team801.robot.RobotMap;
import org.usfirst.frc.team801.robot.commands.DumperIdle;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Dumper extends Subsystem {

	public static CANTalon dumMotor = RobotMap.dumper;
	public static Servo opendoor = RobotMap.openDoor;

	
	public Dumper(){
		
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new DumperIdle());		
	}
	
	public void startUp(){
    	dumMotor.set(1.0);
//    	Timer.delay(0.3);
//    	gatMotor.set(0.0);
    }
   	public void startDown(){
    	dumMotor.set(-1.0);
//    	Timer.delay(0.3);
//    	gatMotor.set(0.0);
    }
	public void stop(){
   		dumMotor.set(0.0);
   	}
	
	public void openDoor(){
		opendoor.set(0.85);
	}
	public void closeDoor(){
		opendoor.set(0.2);
	}
	public double getServoValue(){
		return opendoor.get();
	}
	
	
}
