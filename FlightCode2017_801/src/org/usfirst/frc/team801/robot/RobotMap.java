package org.usfirst.frc.team801.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.usfirst.frc.team801.robot.Utilities.Adis16448_IMU;
import org.usfirst.frc.team801.robot.Utilities.BufferedWriterFRC;

import com.ctre.CANTalon;

import SwerveClass.SwerveDrive;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.internal.HardwareTimer;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap{
	//Gather
	public static CANTalon gather;
	public static CANTalon lift;
	//Chassis
	public static CANTalon rightFrontd;
	public static CANTalon leftFrontd;
	public static CANTalon backLeftd;
	public static CANTalon backRightd;
//	
	public static CANTalon rightFrontt;
	public static CANTalon leftFrontt;	
	public static CANTalon backLeftt;
	public static CANTalon backRightt;
	
	public static SwerveDrive swerveDrive;
	
	public static final PowerDistributionPanel pdp = new PowerDistributionPanel();


	//Wench
	//	public static CANTalon wenchMotor = new CANTalon(8);
	
	//Dumper
	public static CANTalon dumper;
	
	
	//Gear
	public static Servo gearMotor1;
	public static Servo gearMotor2;

	
	//Shooter
	public static CANTalon shoot1;
	public static CANTalon shoot2;
	
 	//IMU
	
	public static Adis16448_IMU imu;
	
	//Sensors
	public static AnalogInput ultraSonic;

	//Gearpicker
	public static CANTalon gearPicker1;
	public static CANTalon gearPicker2;
	
	public static DigitalInput gearPickerEncIn;
	public static DigitalOutput gearPickerEncOut;
		
	public static Relay lightRelay;
	public static Servo openDoor;

	//Data Logger
	public static BufferedWriterFRC logFileCreator;
	public static HardwareTimer timer;
	public static final File logPath = new File("/u/logsKnox/");
	
	public static void init(){
		//Lights
		lightRelay = new Relay(0);
		
		//Actions
		ultraSonic = new AnalogInput(0);

		//IMU setup
		imu = new Adis16448_IMU();
		imu.calibrate();
		imu.reset();
		
		//
		//Gather
		gather = new CANTalon(Constants.GATHER_MOTOR);
		lift = new CANTalon(Constants.LIFT_MOTOR);
		//Chassis Init
		//DriveMotors
		rightFrontd = new CANTalon(Constants.RIGHT_FRONTD);
		leftFrontd = new CANTalon(Constants.LEFT_FRONTD);
		backLeftd = new CANTalon(Constants.BACKLEFT_FRONTD);
		backRightd = new CANTalon(Constants.BACKRIGHT_FRONTD);	
		
		//TurnMotors
		rightFrontt = new CANTalon(Constants.RIGHT_FRONTT);
//		rightFrontt.reverseOutput(true);
		leftFrontt = new CANTalon(Constants.LEFT_FRONTT);
		backLeftt = new CANTalon(Constants.BACKLEFT_FRONTT);
		backRightt = new CANTalon(Constants.BACKRIGHT_FRONTT);

		swerveDrive = new SwerveDrive(rightFrontd, leftFrontd, backLeftd, backRightd,
												rightFrontt,leftFrontt,backLeftt,backRightt,10);
		
		//Dumper
		dumper = new CANTalon(Constants.DUMPER_MOTOR);
		
		//Gear
		gearMotor1 = new Servo(0);
		gearMotor2 = new Servo(1);
		
		openDoor = new Servo(2);
		
		//Shooter
//		shoot1 = new CANTalon(Constants.SHOOT_MOTOR1);
//		shoot2 = new CANTalon(Constants.SHOOT_MOTOR2);
		
//		//gearpicker
//		gearPicker1 = new CANTalon(Constants.GEARPICKER_MOTOR1);
//		gearPicker2 = new CANTalon(Constants.GEARPICKER_MOTOR2);
//		
//		gearPickerEncIn = new DigitalInput(0);
//		gearPickerEncOut = new DigitalOutput(1);
		

	}
	public static void dataWriterInit()
	{					
		//Data Logger
		if(Constants.LOGGING_ENABLE)
		{
			timer = new HardwareTimer();
			if(logPath.isDirectory())
			{
				System.out.println("I think a USB drive is mounted as U");		
//				System.out.println("Size " + logPath.getFreeSpace());
				logFileCreator = createLogFile(logPath);
			}
			else
			{
				System.out.println("No USB Drive mounted");
				logFileCreator = null;
			}
		} //end Data Logger
	}
	private static BufferedWriterFRC createLogFile(File path)
	{
		LocalDateTime dateTime = LocalDateTime.now();
		File outFile = new File(path.toString() + "/log_" + dateTime.format(DateTimeFormatter.ofPattern("uu_MM_dd_HH_mm_ss")) + ".txt");
		try
		{
			BufferedWriterFRC w = new BufferedWriterFRC(new OutputStreamWriter(new FileOutputStream(outFile)));
			System.out.println("Log created at " + path.getName() + "/" + outFile.getName());
			return w;
		}
		catch (FileNotFoundException e)
		{
			System.out.println("WARNING: No drive available. If drive is mounted, try restarting the roboRIO");	
			return null;
		}
		//return w;
	}
}


