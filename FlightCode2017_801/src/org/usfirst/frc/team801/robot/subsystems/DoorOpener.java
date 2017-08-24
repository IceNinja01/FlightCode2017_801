package org.usfirst.frc.team801.robot.subsystems;

import org.usfirst.frc.team801.robot.RobotMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DoorOpener extends Subsystem {

	public static Servo doorMotor = RobotMap.openDoor;



    // Put methods for controlling this subsystem
    // here. Call these from Commands.

public DoorOpener(){
		
	}
	
	@Override
	protected void initDefaultCommand() {
//		setDefaultCommand(new CloseDoorMotor());
	}
	
	public void startUp(){
    	doorMotor.set(0.85);


    }
   	public void startDown(){
    	doorMotor.set(0.2);

    }
   	
	public void stop(){
    
   	}

	public double getMotorValues() {
		return doorMotor.get();
	}
}

