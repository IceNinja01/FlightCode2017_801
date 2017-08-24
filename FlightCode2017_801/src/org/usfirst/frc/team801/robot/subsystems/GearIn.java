package org.usfirst.frc.team801.robot.subsystems;

import org.usfirst.frc.team801.robot.RobotMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearIn extends Subsystem {

	public static Servo gi1Motor = RobotMap.gearMotor1;
	public static Servo gi2Motor = RobotMap.gearMotor2;



    // Put methods for controlling this subsystem
    // here. Call these from Commands.

public GearIn(){
		
	}
	
	@Override
	protected void initDefaultCommand() {
//		setDefaultCommand(new GearOutMotor());
	}
	
	public void startUp(){
    	gi1Motor.set(0.85);
    	gi2Motor.set(0.85);

//    	Timer.delay(0.3);
//    	gatMotor.set(0.0);
    }
   	public void startDown(){
    	gi1Motor.set(0.2);
    	gi2Motor.set(0.2);

//    	Timer.delay(0.3);
//    	gatMotor.set(0.0);
    }
   	
   	public double[] getMotorValues(){
		double[] motorValues= new double[2];
		motorValues[0]=gi1Motor.get();
		motorValues[1] = gi2Motor.get();
		return motorValues;
   		
   	}
   	
	public void stop(){
    
   	}
}

