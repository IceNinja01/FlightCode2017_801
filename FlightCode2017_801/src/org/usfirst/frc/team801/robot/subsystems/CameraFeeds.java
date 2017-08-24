package org.usfirst.frc.team801.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team801.robot.Constants;
import org.usfirst.frc.team801.robot.vision.GripPipelineRGBc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraFeeds extends Subsystem { 
	public static Button test1;
	public static Button test2;
	public double centerPointX;
	public double centerX;
	public Object imgLock;
	public boolean runProcessing = false;
	private GripPipelineRGBc pipeline;
	public int centerY;
	public double skew;
	public boolean allowCam1 = true;
	private double aspectRatio =0.4; //aspect ratio of the peg retro-reflective tape
	
	public CameraFeeds(){//default start the cam0	
		Object imgLock = new Object();
	    	
	    	Thread t = new Thread(() -> {
	    		pipeline = new GripPipelineRGBc();  		
	    		UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
	    		camera1.setExposureAuto();
//	    		camera1.setExposureManual(100);
	            camera1.setResolution(320, 240);
//	            camera1.setFPS(20);

	            
	            CvSink cvSink1 = CameraServer.getInstance().getVideo(camera1);
	            CvSource outputStream = CameraServer.getInstance().putVideo("Switcher1", 320, 240);
	            
	            Mat image = new Mat();
	            while(!Thread.interrupted()) {

		  				if (cvSink1.grabFrame(image) == 0) {
							// Send the output the error.
							outputStream.notifyError(cvSink1.getError());
							// skip the rest of the current iteration
							continue;
		  				}
		  				//draw a white box around center line
//		                  Imgproc.rectangle(image, new Point(50.0,50.0), new Point(100.0,100.0), new Scalar(0, 0, 255), 2);				
		  				if(runProcessing){
	      					pipeline.process(image);
	      					
					    	if (!pipeline.filterContoursOutput().isEmpty()) {

	      						if(pipeline.filterContoursOutput().size()>=2){	
//		      							int n = pipeline.filterContoursOutput().size();
//		      							int[] aspectVector = new int[n];
//		      								for(int i=0; i<n;i++){
//		      									Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
//		      									double ratio = r.width/r.height;
//		      									if((ratio <= aspectRatio*1.3) && (ratio >= aspectRatio*0.7)){//roughly 20% of ratio
//		      										aspectVector[i]=1;
//		      									}
//		      									else{
//		      										aspectVector[i]=0;
//		      									}		
//		      								}
//		      							int p1 = findFirstNotNull(aspectVector, 0);
//		      							int p2 = findFirstNotNull(aspectVector, p1+1); 	
	      							Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));//biggest object first
	      					        Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
	      					        if(r1.x>r2.x){//checks to make sure that r1 is on the left, if not switch to the right
	      					        	r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	      						        r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
	      					        }
	      					        synchronized (imgLock) {
      					            	centerY = ((r1.y - r2.y)-240)/2;
      					            	centerX = ((r1.x - (r2.x +r2.width))-320)/2;
      					            	skew = Math.atan2(centerY,centerX)*180.0/Math.PI;
      					            	centerPointX = (r1.x + (r2.x +r2.width))/2;
      					                centerPointX = centerPointX - (320.0/2 + Constants.camera1Bias);	
      									Imgproc.rectangle(image, new Point(r1.x, r1.y), new Point(r2.x + r2.width, r2.y + r2.height), new Scalar(0, 0, 255), 2); 
      									Imgproc.rectangle(image, new Point(160.0 , 0.0), new Point(190.0 , 240.0), new Scalar(255, 255, 255), 2);
	      					         }
	      					}
					    }
	  				}
                outputStream.putFrame(image);
            }
        	try {
				Thread.sleep(5);
			} catch (Exception e) {

				e.printStackTrace();
			}	            
        });t.start();
        synchronized (imgLock) {
        	SmartDashboard.putNumber("CenterX",centerX);
        }		
	}
	
	public boolean getCamera(){
		return allowCam1;
	}
	public void switchCamera(){
		allowCam1 = !allowCam1;
		SmartDashboard.putBoolean("Camera", allowCam1);
	}
	
	public boolean getProcessing(){
	return runProcessing;
	}
	public void enableProcessing() {
		runProcessing = !runProcessing;
	}
	
	public double getCenterX(){
		SmartDashboard.putNumber("CenterX",centerPointX);
		return centerPointX;
	}
	public double getSkewAngle(){
		SmartDashboard.putNumber("SkewAngle",skew);
		return skew;
	}
	@Override
	protected void initDefaultCommand() {

	}
	public int findFirstNotNull(int[] arr, int startIndex){
		int index=0; 
		for(int i=startIndex;i<arr.length;i++){
		  if(arr[i]!=0){
		    index= i;
		    break;
		  }
		}
		return index;
	}
	
}
