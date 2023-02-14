package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import Team4450.Lib.FXEncoder;
// import Team4450.Lib.SynchronousPID;
// import Team4450.Lib.Util;
// import Team4450.Robot23.commands.autonomous.AutoDrive.Pid;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    // pseudo motors and port. CANSparkMax is what we are actually using, but I do
    // not know the encoders
    private WPI_TalonFX extensionMotor = new WPI_TalonFX(0);
    private FXEncoder extensionMotorEncoder = new FXEncoder(extensionMotor);
    private WPI_TalonFX rotationMotor = new WPI_TalonFX(0);
    private FXEncoder rotationMotorEncoder = new FXEncoder(rotationMotor);

    // Actual motors
    private CANSparkMax CANextensionMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder CANextensionEncoder = CANextensionMotor.getEncoder();
    private CANSparkMax CANrotationMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder CANrotationEncoder = CANrotationMotor.getEncoder();

    private double extensionPower, rotationPower; // Debugging purposes

    // PLACEHOLDER VALUES
    private int minExtensionEncoderCount = 0, maxExtensionEncoderCount = 100, currentExtensionEncoderCount;
    private int minRotationEncoderCount = 0, maxRotationEncoderCount = 100, currentRotationEncoderCount;

    public Arm() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    public void initialize() {
        resetToZero();
    }

    @Override
    public void periodic() {
    }

    private void updateDS() {
        double extensionPercent = currentExtensionEncoderCount * 100.0f / maxExtensionEncoderCount;
        double rotationPercent = currentRotationEncoderCount * 100.0f / maxRotationEncoderCount;

        SmartDashboard.putString("Extension is extened by ", extensionPercent + "%");
        SmartDashboard.putNumber("Current extension power is ", extensionPower);
        SmartDashboard.putString("Rotation is rotated by ", rotationPercent + "%");
        SmartDashboard.putNumber("Current rotation power is ", rotationPower);
    }

    // Slowly moves torward touch sensor, intended to be called on initilize to fix any
    // error if the arm and rotation are in the wrong place
    public void resetToZero() {
        extensionMotor.set(-0.1);
        rotationMotor.set(-0.1);
    }

    // Most likely called upon hitting a touch sensor
    public void resetMovementEncoderCount() {
        extensionMotor.set(0);
        currentExtensionEncoderCount = 0;

        updateDS();
    }

    // Most likely called upon hitting a touch sensor
    public void resetRotationEncoderCount() {
        rotationMotor.set(0);
        currentRotationEncoderCount = 0;

        updateDS();
    }

    public void extendArm(double power) {
        currentExtensionEncoderCount = extensionMotorEncoder.get();

        // If the arm is trying to extend forward, but its at the maximum (or above the
        // maximum) encoder count, it stops. It also checks for the opposite, where its
        // trying to go backward, if it's encoder count is equal to or below the minimum
        // it stops. If both are false, power stays the same.
        extensionMotor.set(((extensionMotor.get() < 0 && currentExtensionEncoderCount <= minExtensionEncoderCount)
                || (extensionMotor.get() > 0 && currentExtensionEncoderCount >= maxExtensionEncoderCount)) ? 0 : power);

        extensionPower = power;

        updateDS();
    }

    // Same as extension but for rotation
    public void rotateArm(double power) {
        currentRotationEncoderCount = rotationMotorEncoder.get();

        rotationMotor.set(((rotationMotor.get() < 0 && currentRotationEncoderCount <= minRotationEncoderCount)
                || (rotationMotor.get() > 0 && currentRotationEncoderCount >= maxRotationEncoderCount)) ? 0 : power);

        rotationPower = power;

        updateDS();
    }
}

// IN PROGRESS IDEAS
/*
 * Turn into 2 subsystems to make parallel running easier
 * move "intelligent" logic into a command
 */