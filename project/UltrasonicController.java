package project;

import static project.Resources.*;

/**
 * Controls the robot's movements based on ultrasonic data.
 * <br><br>
 * Control of the wall follower is applied periodically by the UltrasonicController thread in the 
 * while loop in {@code run()}. Assuming that {@code usSensor.fetchSample()} and {@code 
 * processUsData()} take ~20ms, and that the thread sleeps for 50 ms at the end of each loop, then
 * one cycle through the loop is approximately 70 ms. This corresponds to a sampling rate of 1/70ms
 * or about 14 Hz.
 */
public class UltrasonicController implements Runnable {

  
  // Class variables
  private int DEADBAND =Resources.WALL_DIST_ERR_THRESH;
  private static int FWDSPEED = 130;
  private static int FWDSPEEDBANG = 120;
  private int DELTA =130;      
  private int safeDistance = 25;    
  private static double GAIN = 20; 
  private static double closeGAIN = 0.9;
  private static double leftfarGAIN = 2.4;
  private static double rightfarGAIN = 6.6;
  public static double printError;
  
  /**
   * The distance remembered by the {@code filter()} method.
   */
  private int prevDistance;
  
  /**
   * The number of invalid samples seen by {@code filter()} so far.
   */
  private int invalidSampleCount;
  
  /**
   * Buffer (array) to store US samples. Declared as an instance variable to avoid creating a new
   * array each time {@code readUsSample()} is called.
   */
  private float[] usData = new float[usSensor.sampleSize()];
  
  /**
   * The controller type.
   */
  private String type;
  
  /**
   * Constructor for an abstract UltrasonicController. It makes the robot move forward.
   */
  public UltrasonicController(String type) {
    this.type = type;
    leftMotor.setSpeed(MOTOR_HIGH);
    rightMotor.setSpeed(MOTOR_HIGH);
    leftMotor.forward();
    rightMotor.forward();
  }

  /**
   * Process a movement based on the US distance passed in (BANG-BANG style).
   * 
   * @param distance the distance in cm
   */
  public void bangBangController(int distance) {
    
    float distError = Resources.WALL_DIST - distance;// Compute error
    
    if (Math.abs(distError) <= DEADBAND) { /* Within tolerance */
      leftMotor.setSpeed(FWDSPEEDBANG + DELTA);/* 0 bias            */
      rightMotor.setSpeed(FWDSPEEDBANG + DELTA);
      rightMotor.forward();
      leftMotor.forward();
    }
    else if (distError > 0) { /* Too close            */
      // If robot gets closer than comfortable apply more backwards speed
      if (distance <= safeDistance) { 
        leftMotor.setSpeed(FWDSPEEDBANG);/* Speed up inner wheel */
        rightMotor.setSpeed(FWDSPEEDBANG + 2 * DELTA);/* Speed up outer wheel */ 
        rightMotor.backward(); /* Right wheel backwards */ 
        leftMotor.forward();
      }
      else {
        leftMotor.setSpeed(FWDSPEEDBANG + DELTA);      /* Speed up inner wheel */
        rightMotor.setSpeed(FWDSPEEDBANG);/* Constant outer wheel */ 
        rightMotor.forward();
        leftMotor.forward();
      }          
          
    }
    
    
    else { /* Too far                 */
      leftMotor.setSpeed(FWDSPEEDBANG);/* Exactly opposite to above  */
      rightMotor.setSpeed(FWDSPEEDBANG + 2 * DELTA); 
      leftMotor.forward();;
      rightMotor.forward();
    }
  }

/**
   * Process a movement based on the US distance passed in (P style)
   * 
   * @param distance the distance in cm
   */
  public void pTypeController(int distance) {
    
    float distError = Resources.pDIST - distance;// Compute error
    
    
    
    if (Math.abs(distError) <= Resources.pTHRESH) { /* Within tolerance */
      leftMotor.setSpeed(FWDSPEED);/* 0 bias            */
      rightMotor.setSpeed(FWDSPEED);
      rightMotor.forward();
      leftMotor.forward();
     
    }
    else if (distError > Resources.pTHRESH) { /* Too close            */
      /* Speed up outer wheel according to distance error      */
      int rightMotorspeed = (int) (FWDSPEED + closeadjustment(distError)); 
      /* Speed up inner wheel according to distance error     */    
      int leftMotorspeed = (int) (FWDSPEED + adjustment(distError)); 
          
      // Prevent outer wheel speed from going below 0
      if (rightMotorspeed <= 0) {
        rightMotorspeed = 0;
      }
          
      leftMotor.setSpeed(leftMotorspeed);      /* Speed up inner wheel */
      rightMotor.setSpeed(rightMotorspeed);/* Slow outer wheel */ 
      rightMotor.backward();/* Right wheel backwards */ 
      leftMotor.forward();
          
    }
    else { /* Too far                 */
      /* Speed up outer wheel according to distance error      */
      int rightMotorspeed = (int)(FWDSPEED + rightfaradjustment(distError)); 
      /* Slow down inner wheel according to distance error     */
      int leftMotorspeed = (int)(FWDSPEED - leftfaradjustment(distError));
      if (leftMotorspeed <= 0) {
        leftMotorspeed = 0;
      }
      leftMotor.setSpeed(leftMotorspeed);/* Exactly opposite to above  */
      rightMotor.setSpeed(rightMotorspeed); 
      leftMotor.forward();;
      rightMotor.forward();
      
    }
  }
  
/**
 * 
 * A method which takes in the error distance and returns the proportional adjustment to be made
 * to the wheels depending on the error distance for p-type.
 * Specifies adjustment for the inner wheel when the robot is too close to the wall.
 * @param distError
 * @return adjustment i.e the adjustment speed for the inner wheel
 * 
 */
  private static double adjustment(float distError) {
    
    double adjustment = 0;

    adjustment = GAIN * (double) Math.abs(distError);
      
    return adjustment;
    
  }
  /**
   * 
   * A method which takes in the error distance and returns the proportional adjustment to be made
   * to the wheels depending on the error distance for p-type.
   * Specifies adjustment for the outer wheel when the robot is too close to the wall.
   * @param distError
   * @return adjustment i.e the adjustment speed for the outer wheel
   * 
   */ 
  
  private static double closeadjustment(float distError) {
  
    
    double adjustment = 0;
    
    //specifies the distance for the capping the speed
    int minCapDistance = 17;
    
    //if the distance is less than the minimum capping distance, we cap the speed on the wheel.
    if (-distError + Resources.pDIST <= minCapDistance) {
      return Math.abs(closeGAIN * (minCapDistance - Resources.pDIST));
    }
    
    adjustment = closeGAIN * (double) Math.abs(distError);
    
    return adjustment;
    
  }
  
  /**
   * 
   * A method which takes in the error distance and returns the proportional adjustment to be made
   * to the wheels depending on the error distance for p-type.
   * Specifies adjustment for the inner wheel when the robot is too far from the wall.
   * @param distError
   * @return adjustment i.e the adjustment speed for the inner wheel
   * 
   */
  private static double leftfaradjustment(float distError) {
  
    double adjustment = 0;
    //specifies the distance for the capping the speed
    int maxCapDistance = 45;
    //if the distance is more than the maximum capping distance, we cap the speed on the wheel.
    if (Math.abs(distError) + Resources.pDIST >= maxCapDistance) {
      return leftfarGAIN * (maxCapDistance - Resources.pDIST);
    }
    
    adjustment = leftfarGAIN * (double) Math.abs(distError);
  
    return adjustment;
}

  /**
   * 
   * A method which takes in the error distance and returns the proportional adjustment to be made
   * to the wheels depending on the error distance for p-type.
   * Specifies adjustment for the outer wheel when the robot is too far from the wall.
   * @param distError
   * @return adjustment i.e the adjustment speed for the outer wheel
   * 
   */
  
  private static double rightfaradjustment(float distError) {
  
  
  double adjustment = 0;
  
    //specifies the distance for the capping the speed
    int maxCapDistance = 45;
    //if the distance is more than the maximum capping distance, we cap the speed on the wheel.   
    if (Math.abs(distError) + Resources.pDIST >= maxCapDistance) {
      return rightfarGAIN * (maxCapDistance - Resources.pDIST);
    }
    
    adjustment = rightfarGAIN * (double) Math.abs(distError);
  
    return adjustment;
  
}
  /*
   * Samples the US sensor and invokes the selected controller on each cycle (non Javadoc).
   * 
   * @see java.lang.Thread#run()
   */
  public void run() {
    if (type.equals("BangBang")) {
      while(true) {
        bangBangController(readUsDistance());
        Main.sleepFor(POLL_SLEEP_TIME);
      }
    } else if (type.equals("PType")) {
      while(true) {
        pTypeController(readUsDistance());
        Main.sleepFor(POLL_SLEEP_TIME);
      }
    }
  }

  /**
   * Returns the filtered distance between the US sensor and an obstacle in cm.
   * 
{   * @return the filtered distance between the US sensor and an obstacle in cm
   */
  public int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, convert to cm, cast to int, and filter
    return filter((int) (usData[0] * 100.0));
  }
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  int filter(int distance) {
    if (distance >= 255 && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (distance < 255) {
        // distance went below 255: reset filter and remember the input distance.
        invalidSampleCount = 0;
      }
      prevDistance = distance;
      return distance;
    }
  }

}
