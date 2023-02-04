package Team4450.Robot23.subsystems;

import static Team4450.Robot23.Constants.SHOOTER_TALON;
import static Team4450.Robot23.Constants.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;
import Team4450.Robot23.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

/**
 * Shooter subsystem.
 */
public class Shooter extends PIDSubsystem
{
	private boolean			wheelRunning;
    private BooleanSupplier	wr = () -> wheelRunning;

    private WPI_TalonFX     shooterMotor = new WPI_TalonFX(SHOOTER_TALON);

    public FXEncoder        encoder = new FXEncoder(shooterMotor);

    public final double     defaultPower = .50, lowTargetRPM = 3000, highTargetRPM = 4800, maxRPM = 6000;
    private double          currentPower = defaultPower, targetRPM = highTargetRPM, toleranceRPM = 50;
    private DoubleSupplier	tr = () -> targetRPM;

    private static double   kP = .0002, kI = kP / 100, kD = 0;
    private boolean         startUp, highRPM =  true;
    private double          startTime, kS = .498, kV = .108;

    private Channel         channel;
    
    // ks and kv determined by characterizing the shooter motor. See the shooter characterization
    // project.
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(kS, kV);

	public Shooter(Channel channel)
	{
        super(new PIDController(kP, kI, kD));

        shooterMotor.setInverted(true);
    
        getController().setTolerance(toleranceRPM);
		  
        shooterMotor.setNeutralMode(NeutralMode.Coast);

        this.channel = channel;

        addChild("Motor", shooterMotor);

        Util.consoleLog("Shooter created!");
    }    

	@Override
	public void initSendable( SendableBuilder builder )
	{
	    builder.addBooleanProperty("Running", wr, null);
	    builder.addBooleanProperty("PIDEnabled", this::isEnabled, null);
	    builder.addDoubleProperty("TargetRPM", tr, null);
	    builder.addDoubleProperty("ActualRPM", this::getRPM, null);
	}   	

    /**
     * Put shooter into desired initial state when enabled;
     * @param high True to start at high speed, false for low.
     */
    public void initialize(boolean high)
    {
        Util.consoleLog();
        
        stopWheel();

        if (high)
        {
            targetRPM = highTargetRPM;
            highRPM = true;
        } else
        {
            targetRPM = lowTargetRPM;
            highRPM = false;
        }

        updateDS();
    }
	
	// This method will be called once per scheduler run including when disabled.
	@Override
	public void periodic() 
	{
        // Call the base class periodic function so it can run the underlying
        // PID control. We also watch for robot being disabled and stop the
        // wheel (and PID) if running. We also watch for disable/enable transition
        // to reset the shooter power level.

        if (robot.isEnabled())
        {
            super.periodic();

           //Util.consoleLog("current=%.3f", shooterMotor.getStatorCurrent());

            // This code watches motor startup current draw and if too high we
            // assume wheel is jammed by a ball. We stop wheel, back up channel
            // and restart wheel. If no over draw for 1.5 sec we assume good
            // start up and disable this check for rest of wheel run time.

            // Update: The 2022 robot shooter does not ball jam the motor enough to
            // stop it from starting to turn and eject the ball before we can detect
            // a jam and stop. The current draw difference is too small between no jam
            // and jammed to detect reliably and quick enough to stop the ball moving.
            // So current based jam detection will not work. So we set the current level
            // way high to effectively turn the current jam detection off. We will have
            // to use the fixed indexer backup on shooter motor start.

            if (isRunning() && startUp)
            {
                if (shooterMotor.getStatorCurrent() > 200)
                {
                    // If PID controller is running the motor we need to stop the controller
                    // and restart it after backing up.
                    if (isEnabled())
                    {
                        disable();
                        backupIndexer();
                        enable();
                    }
                    else
                    // If not under PID control, we use regular stop/start.
                    {
                        stopWheel();
                        backupIndexer();
                        startWheel();
                    }
                }

                if (Util.getElaspedTime(startTime) > 1.5) startUp = false;
            }
        }
        else
        {
            if (isRunning()) stopWheel();

            targetRPM = lowTargetRPM;
            highRPM = false;
        }
	}

	private void updateDS()
	{
		SmartDashboard.putBoolean("Shooter", wheelRunning);
		SmartDashboard.putBoolean("ShooterHighRPM", highRPM);
	}

	/**
	 * Stop shooter wheel.
	 */
	public void stopWheel()
	{
		Util.consoleLog();
        
        if (isEnabled()) super.disable(); // Turn off PID if enabled.

        shooterMotor.stopMotor();
		
		wheelRunning = false;
		
		updateDS();
	}

    /**
	 * Start shooter wheel turning. Sets current power level to the
     * level specified.
	 * @param power Power level 0.0 to 1.0.
	 */
	public void startWheel(double power)
	{
		Util.consoleLog("%.2f", power);
        
        currentPower = power;

        if (isEnabled()) disable();  // Turn off underlying PID control. 

        // Back the balls down the channel to make sure the wheel is not jammed
        // by a ball.

        // This is now being handled by code in periodic() function that monitors motor
        // current to detect jam and backup only when needed.
        
        // Update: For 2022 bot, current jam detection does not work so fixed backup of
        // indexer must be used.
        backupIndexer();

		shooterMotor.set(currentPower);
		
        wheelRunning = true;
        
        startUp = true;
        startTime = Util.timeStamp();
		
		updateDS();
    }
    
    /**
     * Start shooter wheel turning with current power level.
     */
    public void startWheel()
    {
        startWheel(currentPower);
    }
    
    /**
     * Toggles shooter wheel on/off.
     * @param power Power level to use when starting wheel level 0.0 to 1.0.
     * @return True if result is wheel on, false if off.
     */
    public boolean toggleWheel(double power)
    {
        Util.consoleLog("%.2f", power);

        if (isRunning())
            stopWheel();
        else
            startWheel(power);

        return isRunning();
    }
    
    /**
     * Toggles shooter wheel on/off. Uses current power level when turning on.
     * @return True if result is wheel on, false if off.
     */
    public boolean toggleWheel()
    {
        return toggleWheel(currentPower);
    }

	/**
	 * Returns running state of shooter wheel.
	 * @return True is running.
	 */
	public boolean isRunning()
	{
		return wheelRunning;
    }
    
    /**
     * Returns the current wheel RPM
     * @return The RPM value
     */
    public double getRPM()
    {
        return encoder.getRPM();
    }

    /**
     * Returns the max RPM seen. Depends on regular calls
     * to getRPM.
     * @return The max RPM value.
     */
    public double getMaxRPM()
    {
        return encoder.getMaxRPM();
    }

    // Called by underlying PID control with the output of the PID calculation
    // each time the scheduler calls the periodic function.
    @Override
    protected void useOutput(double output, double setpoint) 
    {
        // Feed forward calculator wants setpoint in rotations per second so we
        // convert our rpm setpoint to rps. Feed forward calculator should give us
        // a voltage that will achieve the rpm we want. PID controller output will
        // adjust the speed to keeep on the rpm and will compensate for rpm being
        // drug down by a ball passing through the shooter.

        double ff = m_shooterFeedforward.calculate(setpoint / 60);
        
        double volts = output + ff;

        //Util.consoleLog("rpm=%.0f  out=%.3f  set=%.3f  ff=%.3f  v=%.3f", getRPM(), output, setpoint, ff, volts);

        shooterMotor.setVoltage(volts);
    }

    // Called by underlying PID control to get the process measurement value each time
    // the scheduler calls the perodic function.
    @Override
    protected double getMeasurement() 
    {
        return getRPM();
    }

    /**
     * Enable PID control to run shooter wheel at current target RPM.
     */
    @Override
    public void enable()
    {
        Util.consoleLog("target rpm=%.0f", targetRPM);

        // This is now being handled by code in periodic() function that monitors motor
        // current to detect jam and backup only when needed.
        
        // Update: For 2022 bot, current jam detection does not work so fixed backup of
        // indexer must be used.
        
        backupIndexer();
        
        setSetpoint(targetRPM);

        super.enable();

        wheelRunning = true;
        
        startUp = true;
        startTime = Util.timeStamp();
		
		updateDS();
    }

    /**
     * Enable PID control to run shooter wheel at specified target RPM.
     * Sets current target RPM to the specified value.
     * @param rpm RPM to set PID controller to follow.
     */
    public void enable(double rpm)
    {
        targetRPM = rpm;

        enable();
    }

    /**
     * Disable PID control of shooter wheel.
     */
    @Override
    public void disable()
    {
        Util.consoleLog();

        super.disable();

        stopWheel();
		
		updateDS();
    }
    
    /**
     * Toggles shooter wheel PID  control on/off.
     * @return True if result is wheel on, false if off.
     */
    public boolean togglePID()
    {
        Util.consoleLog();

        if (isEnabled())
           disable();
        else
           enable();

        return isRunning();
    }
    
    /**
     * Toggles shooter wheel PID target RPM between low and high.
     * @return True if RPM is high, false if low.
     */
    public boolean toggleHighLowRPM()
    {
        Util.consoleLog();

        if (targetRPM == lowTargetRPM)
        {
           targetRPM = highTargetRPM;
           highRPM = true;
        }
        else
        {
           targetRPM = lowTargetRPM;
           highRPM = false;
        }

        updateDS();

        Util.consoleLog("rpm=%.0f", targetRPM);

        return highRPM;
    }

    /**
     * Runs indexer wheel backward before starting shooter motor
     * to move ball down out of contact with shooter wheel.
     */
    private void backupIndexer()
    {
        // Skip backup in auto mode.
        if (robot.isAutonomous()) return;

        Util.consoleLog();

        channel.toggleIndexerDown();

        Timer.delay(.25);   

        channel.stopIndexer();
    }
}
