
package org.usfirst.frc.team801.robot;

import org.usfirst.frc.team801.robot.commands.BlueLightsOn;
import org.usfirst.frc.team801.robot.commands.RedLightsOn;
import org.usfirst.frc.team801.robot.commands.auto.LeftPeg;
import org.usfirst.frc.team801.robot.commands.auto.MiddlePeg;
import org.usfirst.frc.team801.robot.commands.auto.RightPeg;
//import org.usfirst.frc.team801.robot.subsystems.CameraFeeds;
import org.usfirst.frc.team801.robot.subsystems.Chassis;
import org.usfirst.frc.team801.robot.subsystems.DoorOpener;
import org.usfirst.frc.team801.robot.subsystems.Dumper;
import org.usfirst.frc.team801.robot.subsystems.Gather;
import org.usfirst.frc.team801.robot.subsystems.GearIn;
import org.usfirst.frc.team801.robot.subsystems.GearOut;
import org.usfirst.frc.team801.robot.subsystems.GearPicker;
import org.usfirst.frc.team801.robot.subsystems.Lift;
import org.usfirst.frc.team801.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Command lightsCommand;
    Command autonomousCommand;
	SendableChooser<Command> chooser;
	SendableChooser<Command>  chooser1;
	public static GearPicker gearPicker;
	public static DoorOpener doorOpener;
//	public static CameraFeeds camera;
	public static Chassis chassis;
	public static Lift lift;
	public static Gather gather;
	public static Dumper dumper;
	public static Shooter shooter;
	public static GearIn gearIn;
	public static GearOut gearOut;
	public static OI oi;
	public static Preferences prefs;
	public static double angleCmd_DegCon = 0.0;

//	public static DataWriter dataWriter;
//	public static WriteData writeData;

	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
	public void robotInit() {
    	prefs = Preferences.getInstance();
//    	dataWriter = new DataWriter();
    	RobotMap.init();
//    	camera = new CameraFeeds();
		chassis = new Chassis();
		lift = new Lift();
		doorOpener = new DoorOpener();
		gather = new Gather();
		dumper = new Dumper();
		shooter = new Shooter();
		gearIn = new GearIn();
		gearOut = new GearOut();
//		gearPicker = new GearPicker();
    	chooser = new SendableChooser<>();
    	chooser1 = new SendableChooser<>();

        chooser.addDefault("Middle Peg", new MiddlePeg());
        chooser.addObject("Left Peg", new LeftPeg());
        chooser.addObject("Right Peg", new RightPeg());
        SmartDashboard.putData("Auto mode", chooser);
        
        chooser1.addDefault("Blue Lights", new BlueLightsOn());
        chooser1.addObject("Red Lights", new RedLightsOn());
        SmartDashboard.putData("Team Lights", chooser1);

		oi = new OI();
        
//        if(DataWriter.logFile != null)
//        {
//        	writeData.start();
//        }		
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    @Override
	public void disabledInit(){
		RobotMap.lightRelay.set(Relay.Value.kOn);
		RobotMap.lightRelay.set(Relay.Value.kForward);
//		if(DataWriter.logFile != null)
//        {
//			try
//			{
//				DataWriter.logFile.flush();
//			}
//			catch (IOException e)
//			{
//				e.printStackTrace();
//			}
//    	}
    }
	
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		RobotMap.swerveDrive.currentAngle(RobotMap.rightFrontt,0);
		RobotMap.swerveDrive.currentAngle(RobotMap.leftFrontt,1);
		RobotMap.swerveDrive.currentAngle(RobotMap.backLeftt,2);
		RobotMap.swerveDrive.currentAngle(RobotMap.backRightt,3);
		RobotMap.swerveDrive.currentSpeed(RobotMap.rightFrontd,0);
		RobotMap.swerveDrive.currentSpeed(RobotMap.leftFrontd,1);
		RobotMap.swerveDrive.currentSpeed(RobotMap.backLeftd,2);
		RobotMap.swerveDrive.currentSpeed(RobotMap.backRightd,3);
    	prefs = Preferences.getInstance();
    	Constants.FrontRightBias = prefs.getDouble("FrontRightBias", 0.0);
    	Constants.FrontLeftBias = prefs.getDouble("FrontLeftBias", 0.0);
    	Constants.BackRightBias = prefs.getDouble("BackRightBias", 0.0);
    	Constants.BackLeftBias = prefs.getDouble("BackLeftBias", 0.0);
        SmartDashboard.putNumber("IMU", chassis.getGyroAngle());
        Robot.chassis.getUltraDistance();
//        RobotMap.cameraFeed.changeCam();
		RobotMap.lightRelay.set(Relay.Value.kOn);
		RobotMap.lightRelay.set(Relay.Value.kForward);
//        RobotMap.cameraFeed.updateCamera();
//		Robot.camera.getCenterX();
//		Robot.camera.getSkewAngle();
		SmartDashboard.putNumber("Door",Robot.doorOpener.getMotorValues());
		angleCmd_DegCon= Robot.chassis.getGyroAngle();
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    @Override
	public void autonomousInit() {
//    	if(DataWriter.logFile != null)
//        {
//        	DataWriter.logFile.addFRCEvent("Autonomous Start");
//        }
    	lightsCommand = chooser1.getSelected();
    	lightsCommand.start();
        autonomousCommand = chooser.getSelected();
        autonomousCommand.start();
        
		/* String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} */
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
	public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
	public void teleopInit() {
    	lightsCommand = chooser1.getSelected();
    	lightsCommand.start();

		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
//    	if(DataWriter.logFile != null)
//        {
//        	DataWriter.logFile.addFRCEvent("Teleop Start");
//        }
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
	public void teleopPeriodic() {
        Scheduler.getInstance().run();
        SmartDashboard.putNumber("IMU", chassis.getGyroAngle());
//		Robot.camera.getCenterX();
//		Robot.camera.getSkewAngle();
        Robot.chassis.getUltraDistance();
		SmartDashboard.putNumber("Door",Robot.doorOpener.getMotorValues());
		RobotMap.swerveDrive.currentSpeed(RobotMap.rightFrontd,0);
		RobotMap.swerveDrive.currentSpeed(RobotMap.leftFrontd,1);
		RobotMap.swerveDrive.currentSpeed(RobotMap.backLeftd,2);
		RobotMap.swerveDrive.currentSpeed(RobotMap.backRightd,3);

    }
    
    /**
     * This function is called periodically during test mode
     */
    @Override
	public void testPeriodic() {
        LiveWindow.run();
    }
}
