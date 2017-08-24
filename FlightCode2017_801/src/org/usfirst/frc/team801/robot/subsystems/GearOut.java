package org.usfirst.frc.team801.robot.subsystems;

import org.usfirst.frc.team801.robot.RobotMap;
import org.usfirst.frc.team801.robot.commands.gear.GearOutIdle;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearOut extends Subsystem {

	public static Servo go1Motor = RobotMap.gearMotor1;
	public static Servo go2Motor = RobotMap.gearMotor2;



    // Put methods for controlling this subsystem
    // here. Call these from Commands.

public GearOut(){
		
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new GearOutIdle());
	}
	
	public void startUp(){
		go1Motor.set(0.0);
		go2Motor.set(0.0);
//    	Timer.delay(0.3);
//    	gatMotor.set(0.0);
    }
   	public void startDown(){
   		go1Motor.set(1.0);
   		go2Motor.set(1.0);

//    	Timer.delay(0.3);
//    	gatMotor.set(0.0);
    }
	public void stop(){
		go1Motor.set(0.0);
		go2Motor.set(0.0);
   	}
}

