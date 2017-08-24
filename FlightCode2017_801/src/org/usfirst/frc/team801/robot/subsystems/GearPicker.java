package org.usfirst.frc.team801.robot.subsystems;

import org.usfirst.frc.team801.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearPicker extends Subsystem {
	
	private CANTalon gear1 = RobotMap.gearPicker1;
	private CANTalon gear2 = RobotMap.gearPicker2;
	private double getAngle;
	public static DigitalInput encIn = RobotMap.gearPickerEncIn;
	public static DigitalOutput encOut = RobotMap.gearPickerEncOut;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public double getAngle(int i){
       	getAngle = i;
       	getAngle *= 1.241;
    	return getAngle;
    }
    
    public int getEncoder(int i){ 	
		if(encIn.get()){
    		i += 1;
    	}
		return i;
    }
    
    public void startUp(){
    	gear1.set(0.1);
    	gear2.set(0.1);
    }
    
    public void startDown(){
    	gear1.set(-0.1);
    	gear2.set(-0.1);
    }
    
    public void stop(){
    	gear1.set(0.0);
    	gear2.set(0.0);
    }
}

