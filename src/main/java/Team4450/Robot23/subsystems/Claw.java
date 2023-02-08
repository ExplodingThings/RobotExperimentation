package Team4450.Robot23.subsystems;

// import static Team4450.Robot23.Constants.*;

import java.util.HashMap;

// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.FXEncoder;
// import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.commands.autonomous.AutoDrive.Pid;
// import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private WPI_TalonFX clawMotor = new WPI_TalonFX(0); // placeholder deviceNumber value
    private FXEncoder clawMotorEncoder = new FXEncoder(clawMotor);

    double power;
    double elapsedTime;

    public enum ClawState { // Names the different claw motor positions
        FULLY_OPEN, HOLDING_CUBE, HOLDING_CONE
    }

    private ClawState currentClawState; // Used for SmartDashboard

    // Creates a hash map to store the names of the positions and give them a target
    // encoder value, which is set in the constructor
    private HashMap<ClawState, Integer> clawStates = new HashMap<ClawState, Integer>();

    private int encoderCount;

    private Pid pid;
    private SynchronousPID pidController = null;

    private double kP = .0003, kI = .0003, kD = 0; // Stolen from AutoDrive, may be wrong

    public Claw(double power, Pid pid) {
        clawStates.put(ClawState.HOLDING_CUBE, 20); // placeholder encoder count value
        clawStates.put(ClawState.HOLDING_CONE, 60); // placeholder encoder count value

        this.pid = pid;
        this.power = power;
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

    public void initialize() {
        changeClawState(ClawState.FULLY_OPEN);

        updateDS();
    }

    @Override
    public void periodic() {
    }

    private void updateDS() {
        SmartDashboard.putString("Current claw state: ", currentClawState.toString());
    }

    // Called upon hitting the physical "fully open" button to reset encoders and
    // prevent moving further
    public void resetEncoderCount() {
        clawMotor.set(0);
        encoderCount = 0;
    }

    // Changes claw state based on target encoder count inside the HashMap to the
    // relative ClawState
    public void changeClawState(ClawState clawState) {
        currentClawState = clawState;

        double minimumSpeed = 0;
        if (clawState == ClawState.FULLY_OPEN)
            minimumSpeed = -0.1f; // So it doesn't stop too early if encoder count is not far enough, but if it is
                                  // too far the motor is slow enough to stop nearly instantly

        // Create a PID thing to control motor (Currently entirely stolen from
        // AutoDrive, may be wrong)
        if (pid == Pid.on) {
            pidController = new SynchronousPID(kP, kI, kD);

            if (power < 0) {
                pidController.setSetpoint(-encoderCount);
                pidController.setOutputRange(power, 0);
            } else {
                pidController.setSetpoint(encoderCount);
                pidController.setOutputRange(0, power);

                elapsedTime = Util.getElaspedTime();

                power = pidController.calculate(clawMotorEncoder.get(), elapsedTime);
            }
        }

        if (Math.abs(clawMotor.get()) < minimumSpeed) // To prevent the robot from stopping early if encoder count is
                                                      // wrong
            clawMotor.set(minimumSpeed);
    }
}
