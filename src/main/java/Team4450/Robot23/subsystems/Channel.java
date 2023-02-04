package Team4450.Robot23.subsystems;

import static Team4450.Robot23.Constants.*;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Robot23.RobotContainer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Channel subsystem.
 */
public class Channel extends SubsystemBase
{
	private boolean			indexerRunning, feedingBall;
  	
    private BooleanSupplier  ir = () -> indexerRunning;
    private BooleanSupplier  fb = () -> feedingBall;

    private WPI_VictorSPX   indexerMotor = new WPI_VictorSPX(INDEXER_VICTOR);
      
    private double          defaultPower = .30;
    
    private DigitalInput    ballStopSwitch = new DigitalInput(BALL_STOP_SWITCH);
    private AnalogInput     ballStartSensor = new AnalogInput(BALL_START_SENSOR);

    public int              lowestSensorValue, highestSensorValue;

	public Channel()
	{
        indexerMotor.setInverted(true);

        addChild("IndexerMotor", indexerMotor);
        addChild("StopSwitch", ballStopSwitch);
        addChild("StartSwitch", ballStartSensor);

		Util.consoleLog("Channel created!");
	}

	@Override
	public void initSendable( SendableBuilder builder )
	{
	    builder.addBooleanProperty("Running", ir, null);
	    builder.addBooleanProperty("Feeding", fb, null);
	}
    
    /**
     * Put Channel into desired intial state when enabled.
     */
    public void initialize()
    {
        Util.consoleLog();
        
        stopIndexer();
    }
	
	// This method will be called once per scheduler run.
	@Override
	public void periodic() 
	{
        // When indexer is running in the in direction (toward shooter) if the
        // ball stop detection switch goes true, stop the indexer.

        if (isRunning() && indexerMotor.get() > 0 && getBallStopSwitch() && !feedingBall) stopIndexer();

        // If indexer not running and pickup is running and the light sensor on the intake is
        // blocked (ball present), start the indexer to grab ball.

        //if (!isRunning() && RobotContainer.pickup.isRunning() && getBallStartSensor() < 500) startIndexer();

        int sensorValue = getBallStartSensor();

        if (sensorValue > highestSensorValue) highestSensorValue = sensorValue;
        if (sensorValue < lowestSensorValue) lowestSensorValue = sensorValue;
	}

	private void updateDS()
	{
		SmartDashboard.putBoolean("Indexer", indexerRunning);
	}

	/**
	 * Stop indexer wheel.
	 */
	public void stopIndexer()
	{
		Util.consoleLog();
        
        indexerMotor.stopMotor();
		
		indexerRunning = false;
		
		updateDS();
	}

    /**
	 * Start indexer wheel.
	 * @param power Power level -1.0 to 1.0. + is ball IN.
	 */
	public void startIndexer(double power)
	{
		Util.consoleLog("%.2f", power);
		
		indexerMotor.set(power);
		
		indexerRunning = true;
		
		updateDS();
    }
    
    /**
     * Start index wheel up with default power.
     */
    public void startIndexer()
    {
        startIndexer(defaultPower);
    }
    
    /**
     * Toggles indexer on/off.
     * @param power Power level to use when starting wheel -1.0 to 1.0.
     * + is ball UP.
     * @return True if result is wheel on, false if off.
    */
    public boolean toggleIndexer(double power)
    {
        if (isRunning())
            stopIndexer();
        else
            startIndexer(power);

        return isRunning();
    }
    
    /**
     * Toggles indexer wheel on/off. Uses default + power level when turning on. Forward
     * is ball UP.
     * @return True if result is belt on, false if off.
    */
    public boolean toggleIndexerUp()
    {
        return toggleIndexer(defaultPower);
    }
   
    /**
     * Toggles indexer wheel on/off. Uses default - power level when turning on. Backward
     * is ball DOWN.
     * @return True if result is belt on, false if off.
     */
    public boolean toggleIndexerDown()
    {
        return toggleIndexer(-defaultPower);
    }
    
    /**
     * This is an example of how to pass parameters to a runnable. The two funtions above
     * could be done os one, with a parameter for direction or perhaps power. Both ways
     * are legitimate but this shows how to pass a parameter to a runnable if a case
     * surfaces that needs a parameter. See RobotContainer button config method:
     *  instead of:
     * 	.whenPressed(new InstantCommand(channel::toggleIndexerUp, channel));
     *  do this:
	 *	.whenPressed(new InstantCommand(channel.toggleTheIndexer(true), channel));
     * @param up Direction of wheel rotation. True is up, false is down.
     * @return A runnable object suitable for passing to an InstantCommand.
     */
    public Runnable toggleTheIndexer(boolean up)
    {
        Runnable aRunnable = new Runnable() {
            public void run()
            {
                Util.consoleLog("up=%b", up);

                if (up)
                    toggleIndexerUp();
                else
                    toggleIndexerDown();
            }
        };
    
        return aRunnable;
    }
	/**
	 * Returns running state of wheel.
	 * @return True is running.
	 */
	public boolean isRunning()
	{
		return indexerRunning;
    }

    /**
     * Run the indexer wheel just long enough to pop the top ball
     * into the shooter wheel. This function should be run from
     * a NotifierCommand so it is run in a separate thread.
     */
    public void feedBall()
    {
        Util.consoleLog();

        // Can't feed a ball if shooter wheel is not running.
        if  (!RobotContainer.shooter.isRunning()) return;

        feedingBall = true;

        startIndexer();

        Timer.delay(.75);   // set time so one ball is fed.

        stopIndexer();
    
        feedingBall = false;
    }

    /**
     * Get the state of the ball stop detection switch.
     * @return True if ball contacting switch, false if not.
     */
    public boolean getBallStopSwitch()
    {
        // Invert as switch port returns true when not in contact with ball.
        return !ballStopSwitch.get();
    }

    public int getBallStartSensor()
    {
        return ballStartSensor.getValue();
    }
}
