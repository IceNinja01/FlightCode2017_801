package org.usfirst.frc.team801.robot;

import org.usfirst.frc.team801.robot.Utilities.XBOXJoystick;
import org.usfirst.frc.team801.robot.commands.chassis.Fast;
import org.usfirst.frc.team801.robot.commands.chassis.PegGearCMD;
import org.usfirst.frc.team801.robot.commands.chassis.Slow;
import org.usfirst.frc.team801.robot.commands.gather.GatherDown;
import org.usfirst.frc.team801.robot.commands.gather.GatherUp;
import org.usfirst.frc.team801.robot.commands.gear.GearInMotor;
import org.usfirst.frc.team801.robot.commands.gear.GearOutMotor;
import org.usfirst.frc.team801.robot.commands.CloseDoorMotor;
import org.usfirst.frc.team801.robot.commands.DumpBalls;
import org.usfirst.frc.team801.robot.commands.OpenDoorMotor;
import org.usfirst.frc.team801.robot.commands.ShooterStart;
import org.usfirst.frc.team801.robot.commands.SwitchCamera;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public XBOXJoystick driver = new XBOXJoystick(0);
    public XBOXJoystick manip = new XBOXJoystick(1);
	//Driver buttons
	public Button gatherUp = new JoystickButton(driver, 6);
	public Button gatherDown = new JoystickButton(driver, 8);
	public Button switchCam = new JoystickButton(driver,1);
	public Button goFast = new JoystickButton(driver,3);
	public Button pegGear = new JoystickButton(driver,5);
	//Manipulator buttons
	public Button shootBalls = new JoystickButton(manip, 8);
	public Button dump = new JoystickButton(manip, 5);
	public Button gearOpen = new JoystickButton(manip, 4);
	public Button gearClose = new JoystickButton(manip, 3);
	public Button openDoor = new JoystickButton(manip,1);
	public Button closeDoor = new JoystickButton(manip,2);
	public Button gearPickerb = new JoystickButton(manip,7);

	

	
	public OI(){
		
    	gatherUp.whileHeld(new GatherUp());
    	gatherDown.whileHeld(new GatherDown());
    	dump.whileHeld(new DumpBalls());
    	switchCam.toggleWhenPressed(new SwitchCamera());
    	goFast.whenPressed(new Fast());
    	goFast.whenReleased(new Slow());

    	pegGear.whenPressed(new PegGearCMD());
    	
    	openDoor.whenPressed(new OpenDoorMotor());
    	closeDoor.whenPressed(new CloseDoorMotor());
    	shootBalls.whileHeld(new ShooterStart());
    	gearOpen.whenPressed(new GearInMotor());
    	gearClose.whileHeld(new GearOutMotor());
//    	gearPickerb.whenPressed(new GearPickerDown(447));
//    	gearPickerb.whenReleased(new GearPickerUp(447));
	}
}

