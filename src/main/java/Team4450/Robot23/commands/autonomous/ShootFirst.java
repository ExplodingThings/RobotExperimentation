package Team4450.Robot23.commands.autonomous;

import Team4450.Lib.LCD;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Robot23.RobotContainer;
import Team4450.Robot23.subsystems.Channel;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.Shooter;

import static Team4450.Robot23.Constants.*;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Shoot loaded ball then drive out of starting area.
 */
public class ShootFirst extends CommandBase
{
	private final DriveBase driveBase;

    private SequentialCommandGroup	commands = null;
	private Command					command = null;
	private	Pose2d					startingPose;
	private Shooter					shooter;
	private Channel					channel;

	/**
	 * Creates a new ShootFirst autonomous command.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ShootFirst(DriveBase subsystem, Shooter shooter, Channel channel, Pose2d startingPose) 
	{
		Util.consoleLog("x=%.3f  y=%.3f  hdg=%.1f", startingPose.getX(), startingPose.getY(), 
						startingPose.getRotation().getDegrees());
		
		driveBase = subsystem;

		this.startingPose = startingPose;

		this.shooter = shooter;
		this.channel = channel;
			  
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(this.driveBase);
	}
	
	/**
	 * Called when the command is initially scheduled. (non-Javadoc)
	 * @see edu.wpi.first.wpilibj2.command.Command#initialize()
	 */
	@Override
	public void initialize() 
	{
		Util.consoleLog();
		
		driveBase.setMotorSafety(false);  // Turn off watchdog.
		
	  	LCD.printLine(LCD_1, "Mode: Auto - ShootFirst - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				DriverStation.isFMSAttached(), gameMessage);
		
		// Reset wheel encoders.	  	
	  	driveBase.resetEncodersWithDelay();
	  	
	  	// Set NavX yaw tracking to 0.
	  	RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed during the match.
		double startHeading = startingPose.getRotation().getDegrees();
		
		// This kludge rotates robot orientation to face target.
		if (startHeading < 180) 
			startHeading += 180;
		else 
			startHeading -= 180;

		RobotContainer.navx.setHeading(startHeading);
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startHeading);
			
		// Set Talon ramp rate for smooth acceleration from stop. Determine by observation.
		driveBase.SetCANTalonRampRate(1.0);
			
		// Reset odometry tracking with initial x,y position and heading (passed at constructor). 
		// Robot must be placed in same starting location each time for pose tracking
		// to work. We create a new pose with starting heading computed above.
		startingPose = new Pose2d(startingPose.getX(), startingPose.getY(), Rotation2d.fromDegrees(startHeading));

		driveBase.resetOdometer(startingPose, startHeading);
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		shooter.initialize(true);

		commands = new SequentialCommandGroup();
		
        // First action is shoot the loaded ball.
		// We do this by starting the shooter motor PID at target speed.

		command = new InstantCommand(shooter::enable);
		
		commands.addCommands(command);

		// Now wait for shooter to spin up.

		command = new WaitCommand(2.0);
		
		commands.addCommands(command);

		// Now shoot the loaded ball.

		command = new InstantCommand(channel::feedBall);
		
		commands.addCommands(command);
		
		// Now wait for shooter to shoot.

		command = new WaitCommand(1.0);
		
		commands.addCommands(command);
		
		// Now disable shooter motor PID.

		command = new InstantCommand(shooter::disable);
		
		commands.addCommands(command);

		// Next action is to backup some encoder counts and stop with brakes on.
		
        command = new AutoDrive(driveBase, -.50, 
                                SRXMagneticEncoderRelative.getTicksForDistance(4.5, DRIVE_WHEEL_DIAMETER), 
								AutoDrive.StopMotors.stop,
								AutoDrive.Brakes.on,
								AutoDrive.Pid.on,
								AutoDrive.Heading.angle);
		
		commands.addCommands(command);
		
		// Launch autonomous command sequence.
		
		commands.schedule();
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 */
	@Override
	public void execute() 
	{
	}
	
	/**
	 *  Called when the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();
		
		Util.consoleLog("final heading=%.2f  Radians=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getHeadingR());
		Util.consoleLog("end ---------------------------------------------------------------");
	}
	
	/**
	 *  Returns true when this command should end. That should be when
	 *  all the commands in the command list have finished.
	 */
	@Override
	public boolean isFinished() 
	{
		// Note: commands.isFinished() will not work to detect the end of the command list
		// due to how FIRST coded the SquentialCommandGroup class. 
		
		if (commands == null)
			return true;
		else
			return !commands.isScheduled();
	}
}
