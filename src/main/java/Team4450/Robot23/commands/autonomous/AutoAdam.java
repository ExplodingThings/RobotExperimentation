package Team4450.Robot23.commands.autonomous;

import java.util.List;

import Team4450.Robot23.RobotContainer;
import Team4450.Robot23.commands.autonomous.AutoDrive.Brakes;
import Team4450.Robot23.commands.autonomous.AutoDrive.StopMotors;
import Team4450.Robot23.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoAdam extends CommandBase {
    private final DriveBase driveBase;
	private SequentialCommandGroup commands = null;
	private Command	command = null;
	private	Pose2d	startingPose;

    public AutoAdam(DriveBase driveBase, Pose2d startingPose) {
        this.driveBase = driveBase;

		this.startingPose = startingPose;

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
		driveBase.setMotorSafety(false);  // Turn off watchdog.
		
	  	// Reset wheel encoders.	  	
	  	driveBase.resetEncodersWithDelay();
	  	
	  	// Set NavX yaw tracking to 0.
	  	RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed all during the match.
		RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees());
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees());
			
		// Set Talon ramp rate for smooth acceleration from stop. Determine by observation.
		driveBase.SetCANTalonRampRate(1.0);
			
		// Reset odometry tracking with initial x,y position and heading (set above) specific to this 
		// auto routine. Robot must be placed in same starting location each time for pose tracking
		// to work.
		Pose2d startPose = driveBase.resetOdometer(startingPose, startingPose.getRotation().getDegrees());
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();
		
		// We will create a trajectory and set the robot to follow it.
    
        DifferentialDriveVoltageConstraint constraint = AutoDriveTrajectory.getVoltageConstraint();

        TrajectoryConfig config = AutoDriveTrajectory.getTrajectoryConfig(constraint);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //                                 // Start at the origin set above
        //                                 startPose,
        //                                 // Pass through these two interior waypoints, making an 's' curve path
        //                                 List.of(
        //                                     new Translation2d(startPose.getX() + 3, startPose.getY() + 1),
        //                                     new Translation2d(startPose.getX() + 6, startPose.getY() + 1)
        //                                 ),
        //                                 // End 4 meters straight ahead of where we started, facing forward
        //                                 new Pose2d(startPose.getX() + 9, startPose.getY(), startPose.getRotation()),
        //                                 // Pass config
        //                                 config);		

        Translation2d currentPos = new Translation2d();

        currentPos = new Translation2d(startPose.getX(), startPose.getY());

        Trajectory robotTrajectory = TrajectoryGenerator.generateTrajectory(
                                    startPose, 
                                    List.of(
                                        // currentPos = new Translation2d(currentPos.getX() + 5, currentPos.getY() + 5),
                                        currentPos = new Translation2d(currentPos.getX(), currentPos.getY() - 2 * 0.8),
                                        currentPos = new Translation2d(currentPos.getX() - 2 * 0.8, currentPos.getY()),
                                        currentPos = new Translation2d(currentPos.getX(), currentPos.getY() + 2 * 0.8),
                                        currentPos = new Translation2d(currentPos.getX() + 2 * 0.8, currentPos.getY()),
                                        currentPos = new Translation2d(currentPos.getX(), currentPos.getY() + 1 * 0.8),
                                        currentPos = new Translation2d(currentPos.getX(), currentPos.getY() - 3.5 * 0.8),
                                        currentPos = new Translation2d(currentPos.getX() + 2 * 0.8, currentPos.getY()),
                                        currentPos = new Translation2d(currentPos.getX(), currentPos.getY() + 4.5 * 0.8),
                                        currentPos = new Translation2d(currentPos.getX() - 5 * 0.8, currentPos.getY()),
                                        currentPos = new Translation2d(currentPos.getX(), currentPos.getY() - 5.5 * 0.8)
                                        ),
                                    new Pose2d(currentPos.getX() + 4.5 * 0.8, currentPos.getY(), new Rotation2d(Math.toRadians(0))),
                                    config);		
        

        //command = new AutoDriveTrajectory(driveBase, exampleTrajectory, StopMotors.stop, Brakes.on);
        //command = new AutoDriveTrajectory(driveBase, RobotContainer.slalom1Trajectory, StopMotors.stop, Brakes.on);
        command = new AutoDriveTrajectory(driveBase, robotTrajectory, StopMotors.stop, Brakes.on);
		
		commands.addCommands(command);
		
		commands.schedule();
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 *  In this model, this command just idles while the Command Group we
	 *  created runs on its own executing the steps (commands) of this Auto
	 *  program.
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
		driveBase.stop();
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
		
		return !commands.isScheduled();
    }

}