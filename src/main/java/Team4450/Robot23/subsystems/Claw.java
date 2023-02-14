package Team4450.Robot23.subsystems;

// import static Team4450.Robot23.Constants.*;

import java.util.HashMap;

// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.FXEncoder;
// import Team4450.Lib.SRXMagneticEncoderRelative;
// import Team4450.Lib.SynchronousPID;
// import Team4450.Lib.Util;
// import Team4450.Robot23.commands.autonomous.AutoDrive.Pid;
// import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private TalonFX clawMotor = new TalonFX(0); // placeholder deviceNumber value
    private FXEncoder clawMotorEncoder = new FXEncoder();

    double elapsedTime;

    public enum ClawState { // Names the different claw motor positions
        MANUAL, FULLY_OPEN, HOLDING_CUBE, HOLDING_CONE
    }

    private ClawState currentClawState; // Used for SmartDashboard

    // Creates a hash map to store the names of the positions and give them a target
    // encoder value, which is set in the constructor
    private HashMap<ClawState, Integer> clawStates = new HashMap<ClawState, Integer>();

    // PSEUDO VALUES
    private int minEncoderCount = 0, maxEncoderCount = 100;

    public Claw() {
        clawStates.put(ClawState.HOLDING_CUBE, 20); // placeholder encoder count value
        clawStates.put(ClawState.HOLDING_CONE, 60); // placeholder encoder count value
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

    public void initialize() {
        changeClawState(ClawState.FULLY_OPEN, 0);

        updateDS();
    }

    @Override
    public void periodic() {
    }

    private void updateDS() {
        SmartDashboard.putString("Most recent claw state: ", currentClawState.toString());
    }

    // Called upon hitting the physical "fully open" button to reset encoders and
    // prevent moving further
    public void resetEncoderCount() {
        clawMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    private int getCurrentEncoderCount() {
        return clawMotorEncoder.get();
    }

    // Changes claw state based on target encoder count inside the HashMap to the
    // relative ClawState. Called by a Pid inside ClawPidCommand
    public void changeClawState(ClawState clawState, double power) {
        currentClawState = clawState;

        double minimumSpeed = 0;
        if (clawState == ClawState.FULLY_OPEN)
            minimumSpeed = -0.1f; // So it doesn't stop too early if encoder count is not far enough, but if it is
                                  // too far the motor is slow enough to stop nearly instantly

        // If the power is slower than the minimum speed, set it to minimum speed,
        // otherwise set it to power. Minimum speed does not have limitClaw() on it
        // because it only happens when claw state is fully open, and in that scenario
        // it moves until it hits the button
        clawMotor.set(TalonFXControlMode.PercentOutput,
                (Math.abs(power) < Math.abs(minimumSpeed)) ? minimumSpeed : limitClaw(power));
    }

    // Called by joystick
    public void changeClawState(double power) {
        currentClawState = ClawState.MANUAL;
        clawMotor.set(TalonFXControlMode.PercentOutput, limitClaw(power));
    }

    private double limitClaw(double power) {
        // If the claw is trying to move forward, but its at the maximum (or above the
        // maximum) encoder count, it stops. It also checks for the opposite, where its
        // trying to go backward, if it's encoder count is equal to or below the minimum
        // it stops. If both are false, power stays the same.
        return ((power < 0 && getCurrentEncoderCount() <= minEncoderCount)
                || (power > 0 && getCurrentEncoderCount() >= maxEncoderCount)) ? 0 : power;
    }
}

// IN PROGRESS IDEAS
/*
 * Look at onTarget of pidController
 * move Pid to a command
 */
