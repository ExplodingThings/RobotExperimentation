package Team4450.Robot23.commands;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.RobotContainer;
import Team4450.Robot23.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
    private DriveBase driveBase;

    private double pitch; // Rotation around x axis
    private double yaw; // Rotation around z axis

    private SynchronousPID pidController;

    AutoBalance(DriveBase driveBase) {
        this.driveBase = driveBase;
    }

    public void initialize() {
        resetYaw();
    }

    private void recalibrateRotation() {
        // Adding 180 because getPitch() and getYaw() go from -180 to 180,but im
        // treating it in the code as 0 to 360, and I don't want to edit it
        pitch = RobotContainer.navx.getAHRS().getPitch() + 180;
        yaw = RobotContainer.navx.getAHRS().getYaw() + 180;
    }

    // USING WHILES BECAUSE WE DONT WANT IT TO CONTINUE UNTIL THAT STAGE IS FINISHED

    public void resetYaw() { // YAW WILL HAVE TO BE CHANGED BECAUSE SWERVE DRIVES DON'T WORK WITH IT
        recalibrateRotation();
        double rotationTarget = 0;
        boolean calculateRotation = false;

        // While the robot rotation is not a multiple of 180 continue rotation
        while (Math.abs(yaw) % 180 != 0)
            if (calculateRotation) {
                // if closer to 180, move to 180, else to 0
                rotationTarget = pidController.calculate((yaw > 90 && yaw < 270) ? yaw + 180 : yaw,
                        Util.getElaspedTime());

                driveBase.curvatureDrive(0, rotationTarget, true);

                recalibrateRotation();
            }

        // Stops all the driveBase motors which prevents further rotation in case they
        // are not fully stopped
        driveBase.setPower(0, 0);

        // If the robot is facing the ramp move forward, otherwise that means the robot
        // is facing away from the ramp (and is rotated 180 degrees) so move backward
        balanceRobot(10, (yaw == 0) ? 1 : -1);
    }

    public void balanceRobot(double leeway, int speed) {
        // In case the robot starts entirely off the ramp. Even if it doesnt start off
        // the ramp it doesn't matter as if its already on the ramp angled past leeway
        // then it skips this, and if its on top of the ramp then it will get off, but
        // the rest will move it back on. Also, why would this be called if we already
        // flat on charging station?
        driveBase.setPower(speed, speed);
        while (Math.abs(pitch) <= leeway) {
            recalibrateRotation();
        }

        // Sets the robots power to move upward the ramp. I do this by making the
        // degrees act as -180 to 180, then checking if its negative or positive. If
        // it's negative the front of the robot is tilted down which means it needs to
        // move backward, if it spositive the front of the robot is tilted up which
        // means it needs to move forward.
        speed = (pitch - 180) > 0 ? 1 : -1;
        driveBase.setPower(speed, speed);

        while (Math.abs(pitch) > leeway) {
            recalibrateRotation();
        }

        // All the qualifications have been completed, so it stops the robot motors.
        driveBase.setPower(0, 0);
    }

    // THIS IS A PRESS AND RUN ONCE COMMAND, ONCE THE PREVIOUS METHOD IS DONE
    // NOTHING HAPPENS. THIS MEANS I DO NOT MAKE A execute(), end(), AND
    // isFinished()
}
