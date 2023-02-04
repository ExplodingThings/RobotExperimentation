package Team4450.Robot23.subsystems;

import static Team4450.Robot23.Constants.*;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.Util;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    // Don't know device number, currently temp value
    private WPI_TalonFX clawMotor = new WPI_TalonFX(0);

    // Names the different claw motor positions
    public enum ClawState {
        FULLY_OPEN, HOLDING_CUBE, HOLDING_CONE
    }

    // Used for SmartDashboard
    ClawState currentClawState;

    // Creates a hash map to store the names of the positions and give the a value,
    // which is set in the constructor
    HashMap<ClawState, Double> clawStates = new HashMap<ClawState, Double>();

    // Used to multiply the percentage (shown as double) into amount of rotations to
    // get to designated position
    int encoderMultiplier = 10;

    // Ties in a double value (as a percentage) to the name, which is taken from the
    // ClawState enum
    public Claw() {
        clawStates.put(ClawState.FULLY_OPEN, 0.0);
        clawStates.put(ClawState.HOLDING_CUBE, 0.2);
        clawStates.put(ClawState.HOLDING_CONE, 0.6);
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

    public void initialize() {
        changeState(ClawState.FULLY_OPEN);

        updateDS();
    }

    @Override
    public void periodic() {
    }

    private void updateDS() {
        SmartDashboard.putString("Current claw state: ", currentClawState.toString());
    }

    private void changeState(ClawState clawState) {
        clawMotor.set(ControlMode.Position, clawStates.get(clawState) * encoderMultiplier);

        currentClawState = clawState;
    }
}
