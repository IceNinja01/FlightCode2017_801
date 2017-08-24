package SwerveClass;

import org.usfirst.frc.team801.robot.Constants;
import org.usfirst.frc.team801.robot.Utilities.Utils;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveTwoMotors implements MotorSafety {
	//Variables to be used are set to private
	private double Max = 0.6;
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
	protected CANTalon d_frontRightMotor;
	protected CANTalon d_frontLeftMotor;
	protected CANTalon d_backLeftMotor;
	protected CANTalon d_backRightMotor;
	protected CANTalon t_frontRightMotor;
	protected CANTalon t_frontLeftMotor;
	protected CANTalon t_rearLeftMotor;
	protected CANTalon t_rearRightMotor;
	protected double temp, STR, FWD, RCW;
	protected double A,B,C,D, max;
	protected double L = 1;
	protected double W = 1;
	protected double R = Math.sqrt(L*L+W*W);
	protected double kP = 0.004;
	protected double kI = 0.00;
	protected double kD = 0.0;
	protected double timeUs;
	private double rightSet,leftSet;
	private double[] oldAngle = {0,0,0,0};
	
	
	public  SwerveDriveTwoMotors(final CANTalon FrontRightDriveMotor,final CANTalon FrontLeftDriveMotor,final CANTalon BackLeftDriveMotor,final CANTalon BackRightDriveMotor,
			final CANTalon FrontRightTurnMotor,final CANTalon FrontLeftTurnMotor,final CANTalon rearLeftTurnMotor,final CANTalon rearRightTurnMotor) {
		
		d_frontRightMotor = FrontRightDriveMotor;
		d_frontLeftMotor  = FrontLeftDriveMotor;
		d_backLeftMotor   = BackLeftDriveMotor;
		d_backRightMotor  = BackRightDriveMotor;
		
		t_frontRightMotor = FrontRightTurnMotor;
		t_frontLeftMotor = FrontLeftTurnMotor;
		t_rearLeftMotor = rearLeftTurnMotor;
		t_rearRightMotor =rearRightTurnMotor;
		t_frontRightMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
		t_frontLeftMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);	
		t_rearLeftMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);	
		t_rearRightMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
//		d_frontRightMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
		

		
		//Turn motors PIDControllers
		source = new PIDSource() {
			
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				
			}
			
			@Override
			public double pidGet() {
				System.out.println("Getting value: " + currentAngle(t_frontRightMotor,0));
				return currentAngle(t_frontRightMotor,0);
			}
			
			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}

		};
		
		t_frontRightMotorController = new PIDController(kP, kI, kD, source, t_frontRightMotor);
		t_frontRightMotorController.setContinuous(true);
		t_frontRightMotorController.setAbsoluteTolerance(5.0);
		t_frontRightMotorController.setInputRange(0, 360);
		t_frontRightMotorController.setOutputRange(-0.5, 0.5);
		t_frontRightMotorController.enable();
		
		//Turn motors PIDControllers
				source1 = new PIDSource() {
					
					@Override
					public void setPIDSourceType(PIDSourceType pidSource) {
						
					}
					
					@Override
					public double pidGet() {
						System.out.println("Getting value: " + currentAngle(t_frontLeftMotor,1));
						return currentAngle(t_frontLeftMotor,1);
					}
					
					@Override
					public PIDSourceType getPIDSourceType() {
						return PIDSourceType.kDisplacement;
					}

				};
				
				t_frontLeftMotorController = new PIDController(kP, kI, kD, source1, t_frontLeftMotor);
				t_frontLeftMotorController.setContinuous(true);
				t_frontLeftMotorController.setAbsoluteTolerance(5.0);
				t_frontLeftMotorController.setInputRange(0, 360);
				t_frontLeftMotorController.setOutputRange(-0.5, 0.5);
				t_frontLeftMotorController.enable();
				//Turn motors PIDControllers
				source2 = new PIDSource() {
					
					@Override
					public void setPIDSourceType(PIDSourceType pidSource) {
						
					}
					
					@Override
					public double pidGet() {
						System.out.println("Getting value: " + currentAngle(t_rearLeftMotor,2));
						return currentAngle(t_rearLeftMotor,2);
					}
					
					@Override
					public PIDSourceType getPIDSourceType() {
						return PIDSourceType.kDisplacement;
					}

				};
				
				t_rearLeftMotorController = new PIDController(kP, kI, kD, source2, t_rearLeftMotor);
				t_rearLeftMotorController.setContinuous(true);
				t_rearLeftMotorController.setAbsoluteTolerance(5.0);
				t_rearLeftMotorController.setInputRange(0, 360);
				t_rearLeftMotorController.setOutputRange(-0.5, 0.5);
				t_rearLeftMotorController.enable();
				
				//Turn motors PIDControllers
				source3 = new PIDSource() {
					
					@Override
					public void setPIDSourceType(PIDSourceType pidSource) {
						
					}
					
					@Override
					public double pidGet() {
						System.out.println("Getting value: " + currentAngle(t_rearRightMotor,3));
						return currentAngle(t_rearRightMotor,3);
					}
					
					@Override
					public PIDSourceType getPIDSourceType() {
						return PIDSourceType.kDisplacement;
					}

				};
				
				t_rearRightMotorController = new PIDController(kP, kI, kD, source3, t_rearRightMotor);
				t_rearRightMotorController.setContinuous(true);
				t_rearRightMotorController.setAbsoluteTolerance(5.0);
				t_rearRightMotorController.setInputRange(0, 360);
				t_rearRightMotorController.setOutputRange(-0.5, 0.5);
				t_rearRightMotorController.enable();
		
		
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
		
		//Calculate Angles and Magnitudes for each motor
		FWD = -AxisY;
		STR = AxisX;
		RCW = rotation;
		temp = FWD*Math.cos(gyroAngle) + STR*Math.sin(gyroAngle);
		STR = -FWD*Math.sin(gyroAngle) + STR*Math.cos(gyroAngle);
		FWD = temp;
		//Perform the following calculations for each new set of FWD, STR, and RCW commands:
		A = STR - RCW*(L/R);
		B = STR + RCW*(L/R);
		C = FWD - RCW*(W/R);
		D = FWD + RCW*(W/R);
		
		double[] wheelSpeeds = new double[4];
	    wheelSpeeds[0] = Math.sqrt(B*B + C*C);
	    wheelSpeeds[1] = Math.sqrt(B*B + D*D);
	    wheelSpeeds[2] = Math.sqrt(A*A + D*D);
	    wheelSpeeds[3] = Math.sqrt(A*A + C*C);
	    
	    double[] wheelAngles = new double[4];
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
	    //TODO Come up with fancy way to turn the inversion of motor polarity
//	    double angleDiff = Math.abs(oldAngle[0] - wheelAngles[0]);
//	    if(angleDiff > 90){
    	rightSet = oldAngle[0] + 90.0;
    	leftSet = oldAngle[0] - 90.0;
	    	if(rightSet > 360){
	    		rightSet -= 360.0;
	    	}
	    	if(leftSet < 0){
	    		leftSet += 360.0;
	    	}
	    if(wheelAngles[0]>rightSet && wheelAngles[0]<leftSet){
	    	d_frontRightMotor.set(wheelSpeeds[0]*Max);
	    	wheelAngles[0] -= 180.0;
	    	if(wheelAngles[0] < 0){
	    		wheelAngles[0] += 360.0;
	    	}
	    }    
	    else{
	    	d_frontRightMotor.set(-wheelSpeeds[0]*Max);
	    }
	    rightSet = oldAngle[1] + 90.0;
    	leftSet = oldAngle[1] - 90.0;
	    	if(rightSet > 360){
	    		rightSet -= 360.0;
	    	}
	    	if(leftSet < 0){
	    		leftSet += 360.0;
	    	}
	    if(wheelAngles[1]>rightSet && wheelAngles[1]<leftSet){
	    	d_frontLeftMotor.set(wheelSpeeds[1]*Max);
	    	wheelAngles[1] -= 180.0;
	    	if(wheelAngles[1] < 0){
	    		wheelAngles[1] += 360.0;
	    	}
	    }    
	    else{
	    	d_frontLeftMotor.set(-wheelSpeeds[1]*Max);
	    }
	    rightSet = oldAngle[2] + 90.0;
    	leftSet = oldAngle[2] - 90.0;
	    	if(rightSet > 360){
	    		rightSet -= 360.0;
	    	}
	    	if(leftSet < 0){
	    		leftSet += 360.0;
	    	}
	    if(wheelAngles[2]>rightSet && wheelAngles[2]<leftSet){
	    	d_backLeftMotor.set(wheelSpeeds[2]*Max);
	    	wheelAngles[2] -= 180.0;
	    	if(wheelAngles[2] < 0){
	    		wheelAngles[2] += 360.0;
	    	}
	    }    
	    else{
	    	d_backLeftMotor.set(-wheelSpeeds[2]*Max);
	    }
	    rightSet = oldAngle[3] + 90.0;
    	leftSet = oldAngle[3] - 90.0;
	    	if(rightSet > 360){
	    		rightSet -= 360.0;
	    	}
	    	if(leftSet < 0){
	    		leftSet += 360.0;
	    	}
	    if(wheelAngles[3]>rightSet && wheelAngles[3]<leftSet){
	    	d_backRightMotor.set(wheelSpeeds[3]*Max);
	    	wheelAngles[3] -= 180.0;
	    	if(wheelAngles[3] < 0){
	    		wheelAngles[3] += 360.0;
	    	}
	    }    
	    else{
	    	d_backRightMotor.set(-wheelSpeeds[3]*Max);
	    }
		
		//Turn Motors
	    	t_frontRightMotorController.setSetpoint(wheelAngles[0]);
	    
	    	t_frontLeftMotorController.setSetpoint(wheelAngles[1]);
	    
	    	t_rearLeftMotorController.setSetpoint(wheelAngles[2]);
	    
	    	t_rearRightMotorController.setSetpoint(wheelAngles[3]);
	    
	    
		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
      	}
		oldAngle[0] = wheelAngles[0];
		oldAngle[1] = wheelAngles[1];
		oldAngle[2] = wheelAngles[2];
		oldAngle[3] = wheelAngles[3];
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
		
	    if (d_frontRightMotor != null) {
	      d_frontRightMotor.set(0);
	    }
	    if (t_frontRightMotor != null) {
	      t_frontRightMotor.set(0);
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
		   degs = Utils.wrapAngle0To360Deg(degs) - Constants.AngleBias[num];
		   degs = Utils.wrapAngle0To360Deg(degs);
		   SmartDashboard.putNumber("MotorNum_"+motorNumber+"_Degrees", degs);
		   return degs;
	}

	void updateController(PIDController controller) {
		SmartDashboard.putData("Angle PID", controller);
	    // Comment following when done tuning PID
	    SmartDashboard.putNumber("Angle Error", controller.getError());
	    SmartDashboard.putNumber("Angle Setpoint", controller.getSetpoint());
	    SmartDashboard.putBoolean("Angle On target", controller.onTarget());
	}
	
}
