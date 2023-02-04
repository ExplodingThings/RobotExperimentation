package Team4450.Robot23.subsystems;

import static Team4450.Robot23.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import Team4450.Lib.FunctionTracer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
	
	private WPI_VictorSPX			climberLeftVictor;
    private WPI_TalonSRX            climberRightTalon;

	private MotorControllerGroup	climberDrive;

	//private ValveDA			    climberBrake = new ValveDA(CLIMBER_BRAKE_VALVE);
	private DigitalInput	        climberSwitch = new DigitalInput(CLIMBER_SWITCH);

	// Encoder (regular type) is plugged into dio port n:
	// orange=+5v blue=signal, dio port n+1: black=gnd yellow=signal. 
	//private Encoder			climberEncoder = new Encoder(CLIMBER_ENCODER, CLIMBER_ENCODER + 1, true, EncodingType.k4X);
    
    // SRX magnetic encoder plugged into a CAN Talon.
    private SRXMagneticEncoderRelative  climberEncoder;

	private ValveDA			        mainValve = new ValveDA(MAIN_CLIMBER_VALVE);
    private ValveDA                 auxValve = new ValveDA(AUX_CLIMBER_VALVE);

	private boolean			        brakeEngaged, mainExtended, auxExtended;
	
	public Climber()
	{
		Util.consoleLog();
		
		climberLeftVictor = new WPI_VictorSPX(LEFT_CLIMBER_VICTOR);
		climberRightTalon = new WPI_TalonSRX(RIGHT_CLIMBER_TALON);
	      
		climberLeftVictor.setInverted(true);
	      
	    climberLeftVictor.setNeutralMode(NeutralMode.Brake);
	    climberRightTalon.setNeutralMode(NeutralMode.Brake);

	    climberDrive = new MotorControllerGroup(climberLeftVictor, climberRightTalon);

        climberEncoder = new SRXMagneticEncoderRelative(climberRightTalon, 1);

		climberEncoder.reset();

		addChild("Encoder", climberEncoder);
		addChild("BottomSwitch", climberSwitch);
		addChild("Motors", climberDrive);
		addChild("MainValve", mainValve);
		addChild("AuxValve", auxValve);

		Util.consoleLog("Climber created!");
	}
	
	/**
	 * Put climber into it's initial state when robot enabled.
	 */

	public void initialize()
	{
		Util.consoleLog();

		releaseBrake();
        retractMain();
        retractAux();
	}

	@Override
	public void periodic() 
	{
		// This method will be called once per scheduler run
	}
	
	/**
	 * Set power level for climber motors.
	 * @param power -1 to +1, + is up because we pull the stick back to climb.
	 */
	public void setClimberPower(double power)
	{
		// If trying to go down (-) and switch returns true, we are at bottom so kill the power.
		
		if (power < 0 && climberSwitch.get()) // || climberEncoder.get() <= 0)) 
		{
			climberEncoder.reset();
			power = 0;
		}
		
		// If trying to go up (+) and encoder is at upper limit count, we are the top kill the power.
		if (power > 0 && climberEncoder.get() >= 22300) power = 0;

		climberDrive.set(power);
	}
	
	/**
	 * Stops climber motors.
	 */
	public void stop()
	{
		Util.consoleLog();
		
		climberDrive.stopMotor();
	}

	public boolean getSwitch()
	{
		return climberSwitch.get();
	}

    /**
     * Returns the climber encoder current tick count.
     * @return The tick count.
     */
    public int encoderGet()
    {
        return climberEncoder.get();
    }
	
	/**
	 * Engage the climber brake.
	 */
	public void engageBrake()
	{
		Util.consoleLog();
		
		//climberBrake.SetA();
		
		brakeEngaged = true;
		
		updateDS();
	}
	
	/**
	 * Release the climber brake.
	 */
	public void releaseBrake()
	{
		Util.consoleLog();
		
		//climberBrake.SetB();
		
		brakeEngaged = false;
		
		updateDS();
	}
	
	/**
	 * Returns state of climber brake.
	 * @return True if  brake engaged.
	 */
	public boolean isBrakeEngaged()
	{
		return brakeEngaged;
	}
	
	/**
	 * Toggle state of climber brake.
	 */
	public void toggleBrake()
	{
		Util.consoleLog();
		
		if (brakeEngaged)
			releaseBrake();
		else
			engageBrake();
	}

	private void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Brake", brakeEngaged);
		SmartDashboard.putBoolean("Main Extended", mainExtended);
		SmartDashboard.putBoolean("Aux Extended", auxExtended);
	}
	
	/**
	 * Extend the main clinber arm.
	 */
	public void extendMain()
	{
		Util.consoleLog();
		
		mainValve.SetA();
        
        mainExtended = true;
		
		updateDS();
	}
	
	/**
	 * Retract the main climber arm.
	 */
	public void retractMain()
	{
		Util.consoleLog();
		
        mainValve.SetB();
        
        mainExtended = false;

        stop();
		
		updateDS();
	}
	  
	/**
	 * Toggle between main climber arm extended and retracted.
	 */
	public void toggleDeployMain()
	{
		if (tracing) FunctionTracer.INSTANCE.enterFunction("Climber.toggleDeployMain");

		Util.consoleLog("%b", isMainExtended());
		
		if (isMainExtended())
			retractMain();
		else
		  	extendMain();

		if (tracing) FunctionTracer.INSTANCE.exitFunction("Climber.toggleDeployMain");
    }
	
	/**
	 * Returns extended state of main climber arm.
	 * @return True if arm extended.
	 */
	public boolean isMainExtended()
	{
		return mainExtended;
	}
    	
	/**
	 * Extend the auxilliary clinber arm.
	 */
	public void extendAux()
	{
		Util.consoleLog();
		
		auxValve.SetA();
        
        auxExtended = true;
		
		updateDS();
	}
	
	/**
	 * Retract the auxilliary climber arm.
	 */
	public void retractAux()
	{
		Util.consoleLog();
		
        auxValve.SetB();
        
        auxExtended = false;

        stop();
		
		updateDS();
	}
	  
	/**
	 * Toggle between auxiliiary climber arm extended and retracted.
	 */
	public void toggleDeployAux()
	{
		if (tracing) FunctionTracer.INSTANCE.enterFunction("Climber.toggleDeployAux");

		Util.consoleLog("%b", isAuxExtended());
		
		if (isAuxExtended())
			retractAux();
		else
		  	extendAux();

		if (tracing) FunctionTracer.INSTANCE.exitFunction("Climber.toggleDeployAux");
	}
	
	/**
	 * Returns extended state of main climber arm.
	 * @return True if arm extended.
	 */
	public boolean isAuxExtended()
	{
		return auxExtended;
	}
}
