package org.usfirst.frc.team801.robot.subsystems;

import org.usfirst.frc.team801.robot.RobotMap;
import org.usfirst.frc.team801.robot.commands.ShootIdle;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Shooter extends Subsystem {
	public static CANTalon shoMotor1 = RobotMap.shoot1;
	public static CANTalon shoMotor2 = RobotMap.shoot2;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Shooter(){
		
	}

    @Override
	public void initDefaultCommand() {
    	setDefaultCommand(new ShootIdle());
    }
    
   	public void setMotor(double motorValue){
    	shoMotor1.set(motorValue);
    	shoMotor2.set(motorValue);
    }
   	public void startUp(){
    	shoMotor1.set(-0.45);
    	shoMotor2.set(0.45);
    }
   	public void startDown(){
    	shoMotor1.set(0.45);
    	shoMotor2.set(-0.45);
    }
	public void stop(){
   		shoMotor1.set(0.0);
   		shoMotor2.set(0.0);
   	}
}

