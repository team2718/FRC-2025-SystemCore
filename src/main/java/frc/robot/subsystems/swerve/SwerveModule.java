package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a single module in a swerve drive system, responsible for controlling
 * both the drive and angle motors of the module. This class provides functionality
 * to set the desired state of the module and retrieve its current angle.
 * 
 * <p>Key Features:
 * <ul>
 *   <li>Uses cosine compensation to adjust speed based on angular difference.</li>
 *   <li>Supports continuous input for angle PID control.</li>
 *   <li>Handles conversion factors for drive and angle motors.</li>
 * </ul>
 * 
 * <p>Dependencies:
 * <ul>
 *   <li>{@link TalonFX} for drive motor control.</li>
 *   <li>{@link SparkMax} for angle motor control.</li>
 *   <li>{@link DutyCycleEncoder} for absolute encoder readings.</li>
 *   <li>{@link PIDController} for angle control.</li>
 * </ul>
 * 
 * <p>Usage:
 * <pre>
 * SwerveModuleConfig config = new SwerveModuleConfig(...);
 * SwerveModule module = new SwerveModule(config);
 * SwerveModuleState desiredState = new SwerveModuleState(...);
 * module.setDesiredState(desiredState);
 * </pre>
 * 
 * <p>Configuration:
 * <ul>
 *   <li>Adjust {@code DRIVE_CONVERSION_FACTOR} and {@code ANGLE_CONVERSION_FACTOR} based on motor setup.</li>
 *   <li>Set {@code USE_COSINE_COMPENSATION} to enable or disable speed adjustment.</li>
 * </ul>
 * 
 * @author Your Name
 * @version 1.0
 */
public class SwerveModule {
    private static final boolean USE_COSINE_COMPENSATION = true;

    private static final double DRIVE_CONVERSION_FACTOR = 1.0; // Adjust based on your drive motor's configuration
    private static final double ANGLE_CONVERSION_FACTOR = 1.0; // Adjust based on your angle motor's configuration

    private final TalonFX driveMotor;
    private final SparkMax angleMotor;
    private final AnalogInput absoluteEncoder;
    private final double angleOffset;
    final Translation2d location;
    private final String localName;

    final PIDController angleController = new PIDController(0.25, 0.0, 0.0);

    /**
     * Represents a single swerve module in a swerve drive system.
     * This class encapsulates the functionality for controlling the drive motor,
     * angle motor, and absolute encoder of the module, as well as its physical location
     * and angle offset.
     *
     * @param config The configuration object containing parameters for the swerve module,
     *               including motor IDs, encoder port, angle offset, and physical location.
     *               - {@code driveMotorID}: The ID of the drive motor.
     *               - {@code angleMotorID}: The ID of the angle motor.
     *               - {@code absoluteEncoderPort}: The port for the absolute encoder.
     *               - {@code angleOffset}: The offset for the angle of the module.
     *               - {@code x}: The x-coordinate of the module's location.
     *               - {@code y}: The y-coordinate of the module's location.
     */
    public SwerveModule(String localName, SwerveModuleConfig config) {
        this.driveMotor = new TalonFX(config.driveMotorID);
        this.angleMotor = new SparkMax(0, config.angleMotorID, MotorType.kBrushless);
        this.absoluteEncoder = new AnalogInput(config.absoluteEncoderPort);
        this.angleOffset = config.angleOffset;
        this.location = new Translation2d(config.x, config.y);

        this.localName = localName;

        this.angleController.enableContinuousInput(-180, 180);
    }

    /**
     * Sets the desired state for the swerve module, including speed and angle.
     * The method optimizes the desired state to minimize unnecessary rotation and adjusts
     * the speed using cosine compensation if enabled.
     *
     * @param swerveModuleState The desired state of the swerve module, including speed and angle.
     *                          The state is optimized based on the current angle of the module.
     */
    public void setDesiredState(SwerveModuleState swerveModuleState) {
        swerveModuleState.optimize(getAngle());

        if (USE_COSINE_COMPENSATION) {
            swerveModuleState.speed *= swerveModuleState.angle.minus(getAngle()).getCos();
        }

        driveMotor.setControl(new VelocityVoltage(swerveModuleState.speed * DRIVE_CONVERSION_FACTOR));
        angleMotor.setVoltage(
            angleController.calculate(getAngle().getDegrees(), swerveModuleState.angle.getDegrees())
        );

        SmartDashboard.putNumber(localName + "/angle_raw", getAngleRaw().getDegrees());
        SmartDashboard.putNumber(localName + "/angle", getAngle().getDegrees());
        SmartDashboard.putNumber(localName + "/speed", swerveModuleState.speed);
        SmartDashboard.putNumber(localName + "/angle_setpoint", swerveModuleState.angle.getDegrees());
        SmartDashboard.putNumber(localName + "/speed_setpoint", swerveModuleState.speed);
    }

    private Rotation2d getAngleRaw() {
        double absoluteReading = absoluteEncoder.getVoltage() / RobotController.getVoltage3V3(); // 0 to 1
        double angle = absoluteReading * 360.0; // Convert to degrees and apply offset
        return Rotation2d.fromDegrees(angle);
    }

    /**
     * Retrieves the current angle of the swerve module based on the absolute encoder reading.
     * The absolute encoder provides a value between 0 and 1, which is converted to degrees
     * and adjusted by the configured angle offset.
     *
     * @return A {@link Rotation2d} object representing the current angle of the swerve module in degrees.
     */
    private Rotation2d getAngle() {
        double absoluteReading = absoluteEncoder.getVoltage() / RobotController.getVoltage3V3(); // 0 to 1
        double angle = (absoluteReading * 360.0) - angleOffset; // Convert to degrees and apply offset
        return Rotation2d.fromDegrees(angle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getVelocity().getValueAsDouble() / DRIVE_CONVERSION_FACTOR,
            getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble() / DRIVE_CONVERSION_FACTOR,
            getAngle()
        );
    }
}
