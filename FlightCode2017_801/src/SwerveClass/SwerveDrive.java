package SwerveClass;

import org.usfirst.frc.team801.robot.Constants;
import org.usfirst.frc.team801.robot.Utilities.RollingAverage;
import org.usfirst.frc.team801.robot.Utilities.Utils;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive implements MotorSafety {
	//Variables to be used are set to private
	
	protected MotorSafetyHelper m_safetyHelper;
	PIDSource source;
	PIDSource source1;
	PIDSource source2;
	PIDSource source3;
	protected PIDController t_frontRightMotorController;
	protected PIDController t_frontLeftMotorController;
	protected PIDController t_rearLeftMotorController;
	protected PIDController t_rearRightMotorController;
	public static final double kDefaultExpirationTime = 0.1;
	public static final double kDefaultMaxOutput = 1.0;

	protected double temp, STR, FWD, RCW;
	protected double A,B,C,D, max;
	protected double L = 1;
	protected double W = 1;
	protected double R = Math.sqrt(L*L+W*W);
	protected double kP = 0.005;
	protected double kI = 0.00;
	protected double kD = 0.0;
	protected double timeUs;
	private String motorName[] = {"FrontRight","FrontLeft","BackLeft","BackRight"};
	private double[] rightSet = {0,0,0,0};
	private double[] leftSet = {0,0,0,0};
	private double[] oldAngle = {0,0,0,0};
	private double maxDriveVoltage = 0.7;
	private double maxTurnVoltage = 0.5;
	private double deadBand = 3.0;
	private CANTalon[] driveMotors  = new CANTalon[4];
	private CANTalon[] turnMotors  = new CANTalon[4];
	private PIDController[] pidTurnController  = new PIDController[4];
	private PIDSource[] pidTurnSource  = new PIDSource[4];
	private PIDController[] pidDriveController  = new PIDController[4];
	private PIDSource[] pidDriveSource  = new PIDSource[4];
    private double[] wheelAngles = new double[4];
	private double[] wheelSpeeds = new double[4];
	private double[] angleDiff = new double[4];

	private RollingAverage xavg;
	private RollingAverage yavg;
	private RollingAverage zavg;
	
	
	public  SwerveDrive(final CANTalon FrontRightDriveMotor,final CANTalon FrontLeftDriveMotor,final CANTalon BackLeftDriveMotor,final CANTalon BackRightDriveMotor,
			final CANTalon FrontRightTurnMotor,final CANTalon FrontLeftTurnMotor,final CANTalon rearLeftTurnMotor,final CANTalon rearRightTurnMotor,
			int avgSize) {
		
		driveMotors[0] = FrontRightDriveMotor;
		driveMotors[1]  = FrontLeftDriveMotor;
		driveMotors[2]   = BackLeftDriveMotor;
		driveMotors[3]  = BackRightDriveMotor;
		
		
		turnMotors[0] = FrontRightTurnMotor;
		turnMotors[1] = FrontLeftTurnMotor;
		turnMotors[2] = rearLeftTurnMotor;
		turnMotors[3] =rearRightTurnMotor;
		
		for(int i=0;i<4;i++){
		turnMotors[i].setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
		driveMotors[i].setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		driveMotors[i].configNominalOutputVoltage(+0.0f, -0.0f);
		driveMotors[i].configPeakOutputVoltage(+12.0f, -12.0f);
		driveMotors[i].setProfile(0);
		driveMotors[i].setF(0.026);
		driveMotors[i].setP(0.051);
		driveMotors[i].setI(0);
		driveMotors[i].setD(0);
		driveMotors[i].changeControlMode(TalonControlMode.Speed);
		
		}
		for(int i=0;i<4;i++){
			int j =i;
			pidTurnSource[i] = new PIDSource() {				
				@Override
				public void setPIDSourceType(PIDSourceType pidSource) {				
				}
				@Override
				public double pidGet() {
					return currentAngle(turnMotors[j],j);
				}				
				@Override
				public PIDSourceType getPIDSourceType() {
					return PIDSourceType.kDisplacement;
				}
			};
			pidTurnController[i] = new PIDController(kP, kI, kD, pidTurnSource[i], turnMotors[i]);
			pidTurnController[i].setContinuous(true);
			pidTurnController[i].setAbsoluteTolerance(deadBand);
			pidTurnController[i].setInputRange(0, 360);
			pidTurnController[i].setOutputRange(-maxTurnVoltage, maxTurnVoltage);
			pidTurnController[i].enable();
		}
//		for(int i=0;i<4;i++){
//			int j =i;
//			pidDriveSource[i] = new PIDSource() {				
//				@Override
//				public void setPIDSourceType(PIDSourceType pidSource) {				
//				}
//				@Override
//				public double pidGet() {
//					return currentSpeed(driveMotors[j],j);
//				}				
//				@Override
//				public PIDSourceType getPIDSourceType() {
//					return PIDSourceType.kDisplacement;
//				}
//			};
//			pidDriveController[i] = new PIDController(1.0, kI, kD, pidDriveSource[i], driveMotors[i]);
//			pidDriveController[i].setAbsoluteTolerance(10.0);
//			pidDriveController[i].setInputRange(-100, 100);
//			pidTurnController[i].setOutputRange(-maxDriveVoltage, maxDriveVoltage);
//			pidTurnController[i].enable();
//		}
		
		// Initializes the _avg variables to size avgSize
		xavg = new RollingAverage(avgSize);
		yavg = new RollingAverage(avgSize);
		zavg = new RollingAverage(avgSize);
		
	}
	 /**
	   * Drive method for Swerve wheeled robots.
	   *	  
	   *
	   * <p>This is designed to be directly driven by joystick axes.
	   *
	   * @param AxisX         	The speed that the robot should drive in the X direction. [-1.0..1.0]
	   * @param AxisY         	The speed that the robot should drive in the Y direction. This input is
	   *                  		inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
	   * @param rotation  		The rate of rotation for the robot that is completely independent of the
	   *                  		translation. [-1.0..1.0]
	   * @param gyroAngle 		The current angle reading from the gyro. Use this to implement field-oriented
	   *                  		controls.
	   */
	@SuppressWarnings("ParameterName")
	public void drive(double AxisX, double AxisY, double rotation, double gyroAngle){
		xavg.add(AxisX);
		yavg.add(AxisY);
		zavg.add(rotation);
		//Calculate Angles and Magnitudes for each motor
		FWD = -yavg.getAverage();
		STR = xavg.getAverage();
		RCW = zavg.getAverage();
		double radians = gyroAngle *Math.PI/180.00;
		temp = FWD*Math.cos(radians) + STR*Math.sin(radians);
		STR = -FWD*Math.sin(radians) + STR*Math.cos(radians);
		FWD = temp;
		//Perform the following calculations for each new set of FWD, STR, and RCW commands:
		A = STR - RCW*(L/R);
		B = STR + RCW*(L/R);
		C = FWD - RCW*(W/R);
		D = FWD + RCW*(W/R);
		
	    wheelSpeeds[0] = Math.sqrt(B*B + C*C);
	    wheelSpeeds[1] = Math.sqrt(B*B + D*D);
	    wheelSpeeds[2] = Math.sqrt(A*A + D*D);
	    wheelSpeeds[3] = Math.sqrt(A*A + C*C);
	    
	    //TODO threshold the arctan2 function inputs
	    wheelAngles[0] = Utils.wrapAngle0To360Deg(Math.atan2(B,C)*180/Math.PI);
	    wheelAngles[1] = Utils.wrapAngle0To360Deg(Math.atan2(B,D)*180/Math.PI);
	    wheelAngles[2] = Utils.wrapAngle0To360Deg(Math.atan2(A,D)*180/Math.PI);
	    wheelAngles[3] = Utils.wrapAngle0To360Deg(Math.atan2(A,C)*180/Math.PI);
		
	    //Normalize wheelSpeeds
	    //determine max motor speed
	    max=wheelSpeeds[0]; 
	    if(wheelSpeeds[1]>max){
	    	max=wheelSpeeds[1]; 
	    }
	    if(wheelSpeeds[2]>max){
	    	max=wheelSpeeds[2]; 
	    }
	    if(wheelSpeeds[3]>max){
	    	max=wheelSpeeds[3];
	    }
	    //Divide by max motor speeds
	    if(max>1){
	    	wheelSpeeds[0]/=max; 
	    	wheelSpeeds[1]/=max; 
	    	wheelSpeeds[2]/=max; 
	    	wheelSpeeds[3]/=max;
	    }


		    for(int i=0;i<4;i++){
		    
		    	angleDiff[i]= Math.abs(wheelAngles[i]- oldAngle[i]);

			    if(angleDiff[i] > 90){ //new angle is greater than a 90degree turn, so find shortest path
			    	//reverse translational motors 
			    	driveMotors[i].set(wheelSpeeds[i]*maxDriveVoltage*5400);
			    	
			    	//find new angle
			    	wheelAngles[i] -= 180.0; //subtract 180 degrees
			    	if(wheelAngles[i] < 0){ //wrap to new angle between 0-360
			    		wheelAngles[i] += 360.0;
			    	}
			    	//now the angle is set to move to the shortest path, which is just 180 degrees 
			    	//from the current heading
			    	
			    }    
			    
			    else{
//			    	pidDriveController[i].setSetpoint(-wheelSpeeds[i]*maxDriveVoltage*100.0);
			    	driveMotors[i].set(-wheelSpeeds[i]*maxDriveVoltage*5400);

			    }
				//Turn Motors
			    if(wheelSpeeds[i]>0.1){
			    	pidTurnController[i].setSetpoint(wheelAngles[i]);
			    	oldAngle[i] = wheelAngles[i];
			    }
		    
	    
		    currentAngle(turnMotors[i],i);
		    currentSpeed(driveMotors[i], i);
		    updateController(pidTurnController[i]);
		    
	    }

	    	SmartDashboard.putNumber("Angle", angleDiff[0]);

		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
      	}

	}
	
	public void turnMotors(double angle_CMD){
	    for(int i=0;i<4;i++){
	    	pidTurnController[i].setSetpoint(angle_CMD);
	    }
		
	}
	
	public void turnMotorsDrive(double angle_CMD , double speed){
	    for(int i=0;i<4;i++){
	    	pidTurnController[i].setSetpoint(angle_CMD);
	    	driveMotors[i].set(speed * 5400);
	    }
		
	}
	public void turnMotorsDriveAndRotateAndGyro(double angle_CMD , double AxisY, double rotation, double gyroAngle){
	//Calculate Angles and Magnitudes for each motor
			FWD = -AxisY;
			STR = 0.0;
			RCW = rotation;
			temp = FWD*Math.cos(gyroAngle) + STR*Math.sin(gyroAngle);
			STR = -FWD*Math.sin(gyroAngle) + STR*Math.cos(gyroAngle);
			FWD = temp;
			//Perform the following calculations for each new set of FWD, STR, and RCW commands:
			A = STR - RCW*(L/R);
			B = STR + RCW*(L/R);
			C = FWD - RCW*(W/R);
			D = FWD + RCW*(W/R);
			
		    wheelSpeeds[0] = Math.sqrt(B*B + C*C);
		    wheelSpeeds[1] = Math.sqrt(B*B + D*D);
		    wheelSpeeds[2] = Math.sqrt(A*A + D*D);
		    wheelSpeeds[3] = Math.sqrt(A*A + C*C);
		    //Normalize wheelSpeeds
		    //determine max motor speed
		    max=wheelSpeeds[0]; 
		    if(wheelSpeeds[1]>max){
		    	max=wheelSpeeds[1]; 
		    }
		    if(wheelSpeeds[2]>max){
		    	max=wheelSpeeds[2]; 
		    }
		    if(wheelSpeeds[3]>max){
		    	max=wheelSpeeds[3];
		    }
		    //Divide by max motor speeds
		    if(max>1){
		    	wheelSpeeds[0]/=max; 
		    	wheelSpeeds[1]/=max; 
		    	wheelSpeeds[2]/=max; 
		    	wheelSpeeds[3]/=max;
		    }
		    
		    for(int i=0;i<4;i++){
		    	pidTurnController[i].setSetpoint(angle_CMD);
		    	driveMotors[i].set(wheelSpeeds[i] * 5400);
		    }
	}
	@Override
	public void setExpiration(double timeout) {
		// TODO Auto-generated method stub
		m_safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
		return m_safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return m_safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		for(int i=0;i>4;i++){
		    if (driveMotors[i] != null) {
		      driveMotors[i].set(0);
		    }
		    if (turnMotors[i] != null) {
		      turnMotors[i].set(0);
		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}
	public void brakeOn() {
		for(int i=0;i>4;i++){
		    if (driveMotors[i] != null) {
		      driveMotors[i].enableBrakeMode(true);
		    }
		    if (turnMotors[i] != null) {
		      turnMotors[i].enableBrakeMode(true);
		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}
	public void brakeOff() {
		for(int i=0;i>4;i++){
		    if (driveMotors[i] != null) {
		      driveMotors[i].enableBrakeMode(false);
		    }
		    if (turnMotors[i] != null) {
		      turnMotors[i].enableBrakeMode(false);
		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		m_safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		return m_safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		return "Swerve Drive";
	}
	
  private void setupMotorSafety() {
	    m_safetyHelper = new MotorSafetyHelper(this);
	    m_safetyHelper.setExpiration(kDefaultExpirationTime);
	    m_safetyHelper.setSafetyEnabled(true);
	  }
  
	public double currentAngle(CANTalon motor,int num) {
		   int motorNumber = motor.getDeviceID();
		   double timeUs = motor.getPulseWidthRiseToRiseUs();
		   // Convert timeUs Pulse to angle	   
		   double degs = motor.getPulseWidthRiseToFallUs()*(360.0/timeUs);
		   SmartDashboard.putNumber("RawAngle_"+motorName[num], degs);
		   degs = Utils.wrapAngle0To360Deg(degs) - Constants.AngleBias[num];
		   degs = Utils.wrapAngle0To360Deg(degs);
		   SmartDashboard.putNumber(motorName[num], degs);
		   return degs;
	}
	
	public double currentSpeed(CANTalon motor, int num){
		double speed = motor.getSpeed();
		SmartDashboard.putNumber("Speed"+motorName[num], speed);
		return speed;
	}

	void updateController(PIDController controller) {
		SmartDashboard.putData("Angle PID", controller);
	    // Comment following when done tuning PID
	    SmartDashboard.putNumber("Angle Error", controller.getError());
	    SmartDashboard.putNumber("Angle Setpoint", controller.getSetpoint());
	    SmartDashboard.putBoolean("Angle On target", controller.onTarget());
	}
	public void setMaxDriveVoltage(double setVoltage){
		this.maxDriveVoltage = setVoltage;
	}
}
