package org.usfirst.frc.team801.robot;

public final class Constants {
	public static double FrontRightBias = Robot.prefs.getDouble("FrontRightBias", 0.0);
	public static double FrontLeftBias = Robot.prefs.getDouble("FrontLeftBias", 0.0);
	public static double BackLeftBias = Robot.prefs.getDouble("BackLeftBias", 0.0);
	public static double BackRightBias = Robot.prefs.getDouble("BackRightBias", 0.0);
	public static final double[] AngleBias = {FrontRightBias,FrontLeftBias,BackLeftBias,BackRightBias};
	public static final int DUMPER_MOTOR = 8;

//	public static double ultrakP = Robot.prefs.getDouble("kP", 0.0);
//	public static double ultrakI = Robot.prefs.getDouble("kI", 0.0);
//	public static double ultrakD = Robot.prefs.getDouble("kD", 0.0);

	public static double ultrakP = 0.017;
	public static double ultrakI = 0.000001;
	public static double ultrakD = 0.00015;
	
	//Drive Motors
	public static int RIGHT_FRONTD = 0;
	public static int LEFT_FRONTD = 15;
	public static int BACKLEFT_FRONTD = 14;
	public static int BACKRIGHT_FRONTD = 1;
	//Turn Motors
	public static int RIGHT_FRONTT = 4;
	public static int LEFT_FRONTT = 11;
	public static int BACKLEFT_FRONTT = 10;
	public static int BACKRIGHT_FRONTT = 5;
	//Gather Motor
	public static int GATHER_MOTOR = 9;
	//Wench Motor
	public static int LIFT_MOTOR = 2;
	
	public static final int SHOOT_MOTOR1 = 6;
	public static final int SHOOT_MOTOR2 = 7;
	public static final boolean LOGGING_ENABLE = false;
	
	//Camera
	public static int IMG_WIDTH = 320;
	public static int IMG_HEIGHT = 240;
	public static double camera1Bias = 50;
	public static int GEARPICKER_MOTOR2 = 15;
	public static int GEARPICKER_MOTOR1 = 16;

}
