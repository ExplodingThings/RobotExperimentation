package Team4450.Robot23.subsystems;

import static Team4450.Robot23.Constants.*;

import java.util.function.BooleanSupplier;

//import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.wpilibj.AsynchronousInterrupt;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase
{
	private WPI_VictorSPX	        upperVictor, lowerVictor;

	private MotorControllerGroup    pickupDrive;

	private ValveDA			        pickupValve = new ValveDA(PICKUP_VALVE);

	//private DigitalInput	ballEye = new DigitalInput(BALL_EYE);

	//private AsynchronousInterrupt	interruptHandler;

    private double          pickupPower = .35, interruptTime;
	private boolean			extended = false, pickupRunning = false, interrupted;
    //public static boolean   balleye = false;
     	
    private BooleanSupplier	ex = () -> extended;
    private BooleanSupplier pr = () -> pickupRunning;
	
	public Pickup ()
	{
		Util.consoleLog();

		lowerVictor = new WPI_VictorSPX(LOWER_PICKUP_VICTOR);
		upperVictor = new WPI_VictorSPX(UPPER_PICKUP_VICTOR);

		lowerVictor.setInverted(true);
		upperVictor.setInverted(true);

        pickupDrive = new MotorControllerGroup(lowerVictor, upperVictor);
		addChild("Drive", pickupDrive);
		  
        // Configure interrupt handler for the ballEye optical ball detector. An interrupt
        // handler will run the code (function) we specify when the RoboRio detects a change
        // in the digital signal from the eye.
		
		//interruptHandler = new AsynchronousInterrupt(ballEye, handleInterrupt);
		
        // Listen for a falling edge interrupt. This is because "edge" refers to voltage
        // signal returned by the eye to the digital IO class. When the eye is not blocked
        // it returns 5v which the DIO returns as True. We will invert this for our
        // use as we like to think of not blocked as False and blocked as True. 
        // Since we are interested in the transition between not blocked (5v) and
        // blocked (0v), we interrupt on the falling (voltage) edge of the signal.
		
		//interruptHandler.setInterruptEdges(false, true);
		
		Util.consoleLog("Pickup created!");
    }
 
	@Override
	public void initSendable( SendableBuilder builder )
	{
	    builder.addBooleanProperty("Running", pr, null);
	    builder.addBooleanProperty("Extended", ex, null);
	}   
		
	/**
	 * Put pickup into it's initial state when robot enabled.
	 */

	public void initialize()
	{
		Util.consoleLog();

        retract();
	}
	
    // Called on each run of the scheduler.
    @Override
    public void periodic() 
    {
        // So the ball eye started raising double interrupts for single break of
        // the light beam. Could not figure out why so added some code to record
        // the time of an interrupt and wait 1/4 second before responding to a
        // new interrupt.
		
        if (Util.getElaspedTime(interruptTime) > .25) interrupted = false;
    }

	private void updateDS()
	{
		SmartDashboard.putBoolean("Pickup", pickupRunning);
		SmartDashboard.putBoolean("PickupExtended", extended);
	}
	
	/**
	 * Retract the pickup arm and stop the wheel motor.
	 * Note: retracting the pickup is actually done by extending the cylinder
	 * as the cylinder works opposite to pickup motion.
	 */ 
	public void retract()
	{
		Util.consoleLog();
		
		pickupValve.SetA();
        
        extended = false;
        
		stop();
	}
	
	/**
	 * Extend the pickup arm and start the wheel motor with default power.
	 * Note: extending the pickup is actually done by retracting the cylinder
	 * as the cylinder works opposite to pickup motion.
	 */
	public void extend()
	{
		Util.consoleLog();
		
        pickupValve.SetB();
        
        extended = true;
           
        start(pickupPower);
	}
	  
	/**
	 * Toggle between pickup arm extended and retracted.
	 */
	public void toggleDeploy()
	{
		Util.consoleLog("%b", isExtended());
		
		if (isExtended())
			retract();
		else
		  	extend();
    }
	
	/**
	 * Start pick up wheel and enable optical sensor interrupts.
	 * @param power % power to run wheel motor 0.0->1.0.
	 */
	private void start(double power)
	{
		Util.consoleLog("%.2f", power);
		
		pickupDrive.set(power);
		
		pickupRunning = true;
		
		//interruptHandler.enable();
		
		updateDS();
	}

	/**
	 * Stop wheel motor and disable interrupts.
	 */
	private void stop()
	{
		Util.consoleLog();
		
		pickupDrive.stopMotor();
	
		pickupRunning = false;
		
		// Note, the following function is expensive in terms of the
		// time it takes and will trigger the global watchdog warning
		// and the drivebase motor safety will trigger if set below 1
		// second. No idea why this function takes so long.
		
		//interruptHandler.disable();
	
		updateDS();
	}
	
	/**
	 * Returns extended state of pickup arm.
	 * @return True if arm extended.
	 */
	public boolean isExtended()
	{
		return extended;
	}
	
	/**
	 * Returns state of wheel motor.
	 * @return True if running.
	 */
	public boolean isRunning()
	{
		return pickupRunning;
	}
		
	// Consumer object to handle detection of ball by the ball Eye
	// optical sensor. The sensor generates a hardware interrupt when the 
	// eye is triggered and the handleInterrupt method is called when the
	// interrupt occurs. Keep the length of the code in that method short 
	// as no new interrupts will be reported until handleInterrupt ends.
	// Note we use a delay scheme to ignore interrupts happening in quick
	// succession, as happens with our mechanisim.

	// BiConsumer<Boolean, Boolean > handleInterrupt = (rising, falling) ->
    //     {
    //          if (interrupted) return;

	//     	 Util.consoleLog("ball interrupt(%b,%b)", rising, falling);

    //          channel.startBelt();

    //          Timer.delay(1.5);
             
    //          channel.stopBelt();

    //          interrupted = true;
    //          interruptTime = Util.timeStamp();

	//     	 //Channel channel = (Channel) param;
    //          //channel.intakeBall();    
	// 	};
	
	// private class InterruptHandler extends InterruptHandlerFunction<Object> 
	// {
	//      @Override
	//      public void interruptFired(int interruptAssertedMask, Object param) 
	//      {
    //          if (interrupted) return;

	//     	 Util.consoleLog("ball  interrupt");

    //          channel.startBelt();

    //          Timer.delay(1.5);
             
    //          channel.stopBelt();

    //          interrupted = true;
    //          interruptTime = Util.timeStamp();

	//     	 //Channel channel = (Channel) param;
    //          //channel.intakeBall();
	//      }
	     
//		 public Channel overridableParamter()
//	     {
//			return channel;
//	     }
//	}
	
	/**
	 * Returns state of ball detector electric eye. The electronics returns a high voltage
     * signal (true) when the eye is NOT blocked. But that is inverted from the way we humans 
     * think of this. We expect true when the eye is blocked. So we invert the state of the 
     * eye to match the way humans think of the use of the eye.
     * @return True means ball blocking eye. Will only be true when ball passing eye.
	 */
	// public boolean getBallEye()
	// {
	// 	return !ballEye.get();
	// 
}