
package Team4450.Robot23;

import static Team4450.Robot23.Constants.*;

import java.io.IOException;
import java.nio.file.Path;

import Team4450.Lib.CameraFeed;
import Team4450.Lib.XboxController;
import Team4450.Lib.MonitorCompressor;
import Team4450.Lib.MonitorPDP;
import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Robot23.commands.Climb;
import Team4450.Robot23.commands.TankDrive;
import Team4450.Robot23.commands.Utility.NotifierCommand;
import Team4450.Robot23.commands.autonomous.AutoAdam;
import Team4450.Robot23.commands.autonomous.AutoPathPlanner;
import Team4450.Robot23.commands.autonomous.DriveOut;
import Team4450.Robot23.commands.autonomous.ShootFirst;
import Team4450.Robot23.subsystems.Channel;
import Team4450.Robot23.subsystems.Climber;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.Pickup;
import Team4450.Robot23.subsystems.Shooter;
import Team4450.Robot23.subsystems.ShuffleBoard;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
	// Subsystems.

	public static ShuffleBoard	shuffleBoard;
	public static DriveBase 	driveBase;
	public static Channel		channel;
	public static Pickup		pickup;
	public static Shooter		shooter;
	public static Climber		climber;

	// Subsystem Default Commands.

	private final TankDrive		driveCommand;
	//private final ArcadeDrive	driveCommand;

    // Persistent Commands.

	// Some notes about Commands.
	// When a Command is created with the New operator, its constructor is called. When the
	// command is added to the Scheduler to be run, its initialize method is called. Then on
	// each scheduler run, as long as the command is still scheduled, its execute method is
	// called followed by isFinished. If isFinished it false, the command remains in the
	// scheduler list and on next run, execute is called followed by isFinihsed. If isFinished
	// returns true, the end method is called and the command is removed from the scheduler list.
	// Now if you create another instance with new, you get the constructor again. But if you 
	// are re-scheduling an existing command instance (like the ones above), you do not get the
	// constructor called, but you do get initialize called again and then on to execute & etc.
	// So this means you have to be careful about command initialization activities as a persistent
	// command in effect has two lifetimes (or scopes): Class global and each new time the command
	// is scheduled. Note the FIRST doc on the scheduler process is not accurate as of 2020.
	
	// GamePads. 2 Game Pads use RobotLib XboxController wrapper class for some extra features.
	// Note that button responsiveness may be slowed as the schedulers command list gets longer 
	// or commands get longer as buttons are processed once per scheduler run.
	
	private XboxController			driverPad =  new XboxController(DRIVER_PAD);
	public static XboxController	utilityPad = new XboxController(UTILITY_PAD);

	private AnalogInput			pressureSensor = new AnalogInput(PRESSURE_SENSOR);
	  
	private PowerDistribution	pdp = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);

	// PneumaticsControlModule class controls the PCM. New for 2022.
	private PneumaticsControlModule	pcm = new PneumaticsControlModule(COMPRESSOR);

	// Navigation board.
	public static NavX			navx;

	private Thread      		monitorPDPThread;
	private MonitorCompressor	monitorCompressorThread;
    private CameraFeed			cameraFeed;
    
	// Trajecotries.
    //public static Trajectory    ;

    // List of autonomous programs. Any change here must be reflected in getAutonomousCommand()
    // and setAutoChoices() which appear later in this class.
	private enum AutoProgram
	{
		NoProgram,
		DriveOut,
		ShootFirst,
		AutoAdam,
		AutoPathPlanner
	}

	// Classes to access drop down lists on Driver Station.
	private static SendableChooser<AutoProgram>	autoChooser;
	private static SendableChooser<Pose2d>		startingPoseChooser;

	/**
	 * The container for the robot. Contains subsystems, Opertor Interface devices, and commands.
	 */
	public RobotContainer() throws Exception
	{
		Util.consoleLog();

		// Get information about the match environment from the Field Control System.
      
		getMatchInformation();

		// Read properties file from RoboRio "disk".
      
		robotProperties = Util.readProperties();

		// Is this the competition or clone robot?
   		
		if (robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;
 		
		// Set compressor enabled switch on dashboard from properties file.
		// Later code will read that setting from the dashboard and turn 
		// compressor on or off in response to dashboard setting.
 		
		SmartDashboard.putBoolean("CompressorEnabled", Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault")));

		// Reset PDB & PCM sticky faults.
    
		resetFaults();

		// Create NavX object here since must done before CameraFeed is created (don't remember why).
        // Navx calibrates at power on and must complete before robot moves. Takes ~1 second for 2nd
        // generation Navx ~15 seconds for classic Navx. We assume there will be enough time between
        // power on and our first movement because normally things don't happen that fast.

		navx = NavX.getInstance(NavX.PortType.SPI);

		// Add navx as a Sendable. Updates the dashboard heading indicator automatically.
 		
		SmartDashboard.putData("Gyro2", navx);

		// Invert driving joy sticks Y axis so + values mean forward.
	  
		driverPad.invertY(true);
		
		// Set climber joystick dead zone to reduce twitchyness.
	
		utilityPad.deadZone(.50);

		// Create subsystems prior to button mapping.

		shuffleBoard = new ShuffleBoard();
		driveBase = new DriveBase();
        channel = new Channel();
        shooter = new Shooter(channel);
        pickup = new Pickup();
		climber = new Climber();

		// Create any persistent commands.

		// Set subsystem Default commands.
		
		// Set the default climb command. This command will be scheduled automatically to run
		// every teleop period and so use the utility pad right joy stick to control the climber winch.
		// We pass in DoubleSupplier function on Joystick so the command can read the stick 
		// generically as a DoubleProvider when it runs later (see below).
		
		climber.setDefaultCommand(new Climb(climber, utilityPad.getRightYDS()));

		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the gamepad joy sticks to drive the robot. We pass the GetY()
		// functions on the Joysticks as a DoubleSuppier. The point of this is removing the direct 
		// connection between the Drive and XboxController classes. We are in effect passing functions 
		// into the Drive command so it can read the values later when the Drive command is executing 
		// under the Scheduler. Drive command code does not have to know anything about the JoySticks 
		// (or any other source) but can still read them. We can pass the DoubleSupplier two ways. First
		// is with () -> lambda expression which wraps the getLeftY() function in a DoubleSupplier instance.
		// Second is using the convenience method getRightYDS() which returns getRightY() as a DoubleSupplier. 
		// We show both ways here as an example.

        driveBase.setDefaultCommand(driveCommand = new TankDrive(driveBase, () -> driverPad.getLeftY(),
																			driverPad.getRightYDS()));

		// driveBase.setDefaultCommand(driveCommand = new ArcadeDrive(driveBase, 
		// 															() -> rightStick.GetY(), 
        //                                                             () -> rightStick.GetX(),
        //                                                             () -> rightStick.getJoyStick().getTrigger()));
		   
		// Start the compressor, PDP and camera feed monitoring Tasks.

   		monitorCompressorThread = MonitorCompressor.getInstance(pressureSensor);
   		monitorCompressorThread.setDelay(1.0);
   		monitorCompressorThread.SetLowPressureAlarm(50);
   		monitorCompressorThread.start();
		
   		monitorPDPThread = MonitorPDP.getInstance(pdp);
   		monitorPDPThread.start();
		
		// Start camera server thread using our class for usb cameras.
    
		cameraFeed = CameraFeed.getInstance(); 
		cameraFeed.start();
 		
		// Log info about NavX.
	  
		navx.dumpValuesToNetworkTables();
 		
		if (navx.isConnected())
			Util.consoleLog("NavX version=%s", navx.getAHRS().getFirmwareVersion());
		else
		{
			Exception e = new Exception("NavX is NOT connected!");
			Util.logException(e);
		}
        
        // Configure autonomous routines and send to dashboard.

		setAutoChoices();

		setStartingPoses();

		// Configure the button bindings.
		
        configureButtonBindings();
        
        // Load any trajectory files in a separate thread on first scheduler run.
        // We do this because trajectory loads can take up to 10 seconds to load so we want this
        // being done while we are getting started up. Hopefully will complete before we are ready to
        // use the trajectory. See Robot22B2 for example of how to do this.

		// CANCoder test code.
		if (RobotBase.isSimulation())
		{
			//canCoder.initializeSim();
			//canCoder.reset();
		}
	}

	/**
	 * Use this method to define your button->command mappings.
     * 
     * These buttons are for real robot driver station with 3 sticks and launchpad.
	 * The launchpad makes the colored buttons look like a joystick.
	 */
	private void configureButtonBindings() 
	{
		Util.consoleLog();
	  
		// ------- Driver pad buttons -------------
		
		// For simple functions, instead of creating commands, we can call convenience functions on
		// the target subsystem from an InstantCommand. It can be tricky deciding what functions
		// should be an aspect of the subsystem and what functions should be in Commands...

		// Toggle alternate driving mode.
		new Trigger(() -> driverPad.getRightTrigger())
			.onTrue(new InstantCommand(driveCommand::toggleAlternateDrivingMode));

		// Advance DS tab display.
		new Trigger(() -> driverPad.getPOVAngle(90))
			.onTrue(new InstantCommand(shuffleBoard::switchTab));
 
		// -------- Utility pad buttons ----------
		// Toggle extend Pickup.
		// So we show 3 ways to control the pickup. A regular command that toggles pickup state,
		// an instant command that calls a method on Pickup class that toggles state and finally
		// our special notifier variant that runs the Pickup class toggle method in a separate
		// thread. So we show all 3 methods as illustration but the reason we tried 3 methods is
		// that the pickup retraction action takes almost 1 second (due apparently to some big
		// overhead in disabling the electric eye interrupt) and triggers the global and drivebase
		// watchdogs (20ms). Threading does not as the toggle method is not run on the scheduler thread.
		// Also, any action that operates air valves, there is a 50ms delay in the ValveDA and SA
		// classes to apply power long enough so that the valve slides move far enough to open/close.
		// So any function that operates valves will trigger the watchdogs. Again, the watchdog 
		// notifications are only a warning (though too much delay on main thread can effect robot
		// operation) they can fill the Riolog to the point it is not useful.
		// Note: the threaded command can only execute a runnable (function on a class) not a Command.
		
		// Toggle pickup deployment
		new Trigger(() -> utilityPad.getLeftBumper())
        	//.onTrue(new PickupDeploy(pickup));		
			//.onTrue(new InstantCommand(pickup::toggleDeploy, pickup));
			.onTrue(new NotifierCommand(pickup::toggleDeploy, 0.0, "DeployPickup", pickup));

		// Toggle shooter motor.
		new Trigger(() -> utilityPad.getRightBumper())
			.onTrue(new InstantCommand(shooter::togglePID, shooter));		
		
		// Toggle indexer up direction.
		new Trigger(() -> utilityPad.getPOVAngle(0))
			.onTrue(new InstantCommand(channel.toggleTheIndexer(true), channel));
		
		// Toggle indexer down direction.
		new Trigger(() -> utilityPad.getPOVAngle(180))
			.onTrue(new InstantCommand(channel::toggleIndexerDown, channel));

		// Feed the ball (shoot).
		new Trigger(() -> utilityPad.getRightTrigger())
            .onTrue(new NotifierCommand(channel::feedBall, 0.0, "FeedBall"));

        // Reset odometer.
		new Trigger(() -> driverPad.getBackButton())
    		.onTrue(new InstantCommand(driveBase::zeroOdometer));
        
        // Toggle shooter wheel high/low RPM.
		new Trigger(() -> utilityPad.getYButton())
    		.onTrue(new InstantCommand(shooter::toggleHighLowRPM));

    	// Reset encoders.
		new Trigger(() -> driverPad.getStartButton())
    		.onTrue(new InstantCommand(driveBase::resetEncoders));

		// Toggle position of main climber arms.
		new Trigger(() -> utilityPad.getXButton())
    		.onTrue(new NotifierCommand(climber::toggleDeployMain, 0.0, "DeployMain"));

		// Toggle position of aux climber arms.
		new Trigger(() -> utilityPad.getBButton())
    		.onTrue(new NotifierCommand(climber::toggleDeployAux, 0.0, "DeployAux"));
			
		// Toggle drive CAN Talon brake mode.
		new Trigger(() -> driverPad.getRightBumper())
			.onTrue(new InstantCommand(driveBase::toggleCANTalonBrakeMode));
		
		// Toggle camera feeds. 
		new Trigger(() -> driverPad.getLeftBumper())
    		.onTrue(new InstantCommand(cameraFeed::ChangeCamera));
	}

	/**
	 * Use this to pass the autonomous command(s) to the main {@link Robot} class.
	 * Determines which auto command from the selection made by the operator on the
	 * DS drop down list of commands.
	 * @return The command to run in autonomous
	 */
	public Command getAutonomousCommand() 
	{
		AutoProgram		program = AutoProgram.NoProgram;
		Pose2d			startingPose = BLUE_1;
		Command			autoCommand = null;
		
		Util.consoleLog();

		try
		{
			program = autoChooser.getSelected();

			startingPose = startingPoseChooser.getSelected();
		}
		catch (Exception e)	{ Util.logException(e); }
		
		switch (program)
		{
			case NoProgram:
				autoCommand = null;
				break;
 				
			case DriveOut:
				autoCommand = new DriveOut(driveBase, startingPose);
				break;
 				
			case ShootFirst:
				autoCommand = new ShootFirst(driveBase, shooter, channel, startingPose);
				break;

			case AutoAdam:
				autoCommand = new AutoAdam(driveBase, ADAM_STARTING_POS);
				break;

			case AutoPathPlanner:
				autoCommand = new AutoPathPlanner(driveBase, startingPose);
				break;
		}
        
        // Reset motor deadband for auto.
        driveBase.setPowerDeadBand(.02);

		return autoCommand;
	}
  
    // Configure SendableChooser (drop down list on dashboard) with auto program choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private static void setAutoChoices()
	{
		Util.consoleLog();
		
		autoChooser = new SendableChooser<AutoProgram>();
		
		SendableRegistry.add(autoChooser, "Auto Program");
		autoChooser.setDefaultOption("No Program", AutoProgram.NoProgram);
		autoChooser.addOption("Drive Out", AutoProgram.DriveOut);		
		autoChooser.addOption("Shoot First", AutoProgram.ShootFirst);		
		autoChooser.addOption("Auto Adam", AutoProgram.AutoAdam);
		autoChooser.addOption("Auto Path Planner", AutoProgram.AutoPathPlanner);
				
		SmartDashboard.putData(autoChooser);
	}
  
    // Configure SendableChooser (drop down list on dashboard) with starting pose choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private static void setStartingPoses()
	{
		Util.consoleLog();
		
		startingPoseChooser = new SendableChooser<Pose2d>();
		
		SendableRegistry.add(startingPoseChooser, "Start Position");
		startingPoseChooser.setDefaultOption("Default", DEFAULT_STARTING_POSE);
		startingPoseChooser.addOption("Blue 1", BLUE_1);		
		startingPoseChooser.addOption("Blue 2", BLUE_2);		
		startingPoseChooser.addOption("Blue 3", BLUE_3);		
		startingPoseChooser.addOption("Blue 4", BLUE_4);		

		startingPoseChooser.addOption("Red 1", RED_1);		
		startingPoseChooser.addOption("Red 2", RED_2);		
		startingPoseChooser.addOption("Red 3", RED_3);		
		startingPoseChooser.addOption("Red 4", RED_4);		
				
		SmartDashboard.putData(startingPoseChooser);
	}

	/**
	 *  Get and log information about the current match from the FMS or DS.
	 */
	public void getMatchInformation()
	{
		alliance = DriverStation.getAlliance();
  	  	location = DriverStation.getLocation();
  	  	eventName = DriverStation.getEventName();
	  	matchNumber = DriverStation.getMatchNumber();
	  	gameMessage = DriverStation.getGameSpecificMessage();
    
	  	Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, DriverStation.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
	}
		
	/**
	 * Reset sticky faults in PDP and PCM and turn compressor on/off as
	 * set by switch on DS.
	 */
	public void resetFaults()
	{
		// This code turns on/off the automatic compressor management if requested by DS. Putting this
		// here is a convenience since this function is called at each mode change.
		if (SmartDashboard.getBoolean("CompressorEnabled", true)) 
			pcm.enableCompressorDigital();
		else
			pcm.disableCompressor();
		
		pdp.clearStickyFaults();
		pcm.clearAllStickyFaults();
    }
         
    /**
     * Loads a Pathweaver path file into a trajectory.
     * @param fileName Name of file. Will automatically look in deploy directory.
     * @return The path's trajectory.
     */
    public static Trajectory loadTrajectoryFile(String fileName)
    {
        Trajectory  trajectory;
        Path        trajectoryFilePath;

        try 
        {
          trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + fileName);

          Util.consoleLog("loading trajectory: %s", trajectoryFilePath);
          
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryFilePath);
        } catch (IOException ex) {
          throw new RuntimeException("Unable to open trajectory: " + ex.toString());
        }

        Util.consoleLog("trajectory loaded: %s", fileName);

        return trajectory;
    }

}
