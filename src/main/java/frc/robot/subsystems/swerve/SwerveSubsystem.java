package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.OnboardIMU;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.OnboardIMU.MountOrientation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The SwerveSubsystem class represents a swerve drive system for a robot.
 * It manages four swerve modules (front-left, front-right, back-left,
 * back-right)
 * and uses kinematics to calculate the desired states for each module based on
 * the desired chassis speeds.
 * 
 * <p>
 * Features:
 * <ul>
 * <li>Initialization of swerve modules with specific configurations.</li>
 * <li>Calculation of swerve module states using kinematics.</li>
 * <li>Periodic updates to set the desired state for each module.</li>
 * </ul>
 * 
 * <p>
 * Usage:
 * <ul>
 * <li>Instantiate the subsystem to initialize the swerve modules and
 * kinematics.</li>
 * <li>Set the desired chassis speeds to control the robot's movement.</li>
 * <li>The periodic method automatically updates the module states.</li>
 * </ul>
 * 
 * <p>
 * Dependencies:
 * <ul>
 * <li>SwerveModule: Represents an individual swerve module.</li>
 * <li>SwerveModuleConfig: Configuration for each swerve module.</li>
 * <li>SwerveDriveKinematics: Handles kinematics calculations for swerve
 * drive.</li>
 * <li>ChassisSpeeds: Represents the desired speeds for the robot chassis.</li>
 * </ul>
 */
public class SwerveSubsystem extends SubsystemBase {

    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    SwerveDriveKinematics kinematics;
    SwerveDrivePoseEstimator poseEstimator;

    OnboardIMU imu = new OnboardIMU(MountOrientation.kFlat);

    public Field2d field = new Field2d();

    /**
     * The SwerveSubsystem class is responsible for configuring and managing the
     * swerve drive modules
     * and kinematics for a robot. It initializes four swerve modules (front-left,
     * front-right, back-left,
     * and back-right) with their respective configurations, including motor IDs,
     * encoder ports, angle offsets,
     * and positions on the robot. The subsystem also sets up the swerve drive
     * kinematics using the locations
     * of the modules.
     * 
     * <p>
     * Key functionalities:
     * <ul>
     * <li>Defines configurations for each swerve module, including drive motor ID,
     * angle motor ID,
     * absolute encoder port, angle offset, and position (x, y).</li>
     * <li>Creates instances of SwerveModule for each module using the defined
     * configurations.</li>
     * <li>Initializes the SwerveDriveKinematics object to handle the kinematics of
     * the swerve drive system.</li>
     * </ul>
     * 
     * <p>
     * Usage:
     * This subsystem is typically used in conjunction with a higher-level control
     * system to manage
     * robot movement and orientation using swerve drive principles.
     */
    public SwerveSubsystem() {
        SwerveModuleConfig frontLeftConfig = new SwerveModuleConfig()
                .driveMotorID(1)
                .angleMotorID(2)
                .absoluteEncoderPort(0)
                .angleOffset(49)
                .x(0.5)
                .y(0.5);

        SwerveModuleConfig frontRightConfig = new SwerveModuleConfig()
                .driveMotorID(3)
                .angleMotorID(4)
                .absoluteEncoderPort(1)
                .angleOffset(175.5)
                .x(0.5)
                .y(-0.5);

        SwerveModuleConfig backLeftConfig = new SwerveModuleConfig()
                .driveMotorID(5)
                .angleMotorID(6)
                .absoluteEncoderPort(2)
                .angleOffset(89.5)
                .x(-0.5)
                .y(0.5);

        SwerveModuleConfig backRightConfig = new SwerveModuleConfig()
                .driveMotorID(7)
                .angleMotorID(8)
                .absoluteEncoderPort(3)
                .angleOffset(12.5)
                .x(-0.5)
                .y(-0.5);

        frontLeft = new SwerveModule("Front Left", frontLeftConfig);
        frontRight = new SwerveModule("Front Right", frontRightConfig);
        backLeft = new SwerveModule("Back Left", backLeftConfig);
        backRight = new SwerveModule("Back Right", backRightConfig);

        kinematics = new SwerveDriveKinematics(frontLeft.location, frontRight.location, backLeft.location,
                backRight.location);

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d()); // x,y,heading in radians; Vision measurement std dev, higher=less weight
    }

    public void setDesiredSpeeds(ChassisSpeeds speeds) {
        this.desiredSpeeds = speeds;
    }

    public void setDesiredSpeeds(double vx, double vy, double omega) {
        this.desiredSpeeds = new ChassisSpeeds(vx, vy, omega);
    }

    public void setDesiredSpeedsFieldOriented(double vx, double vy, double omega, double robotAngle) {
        this.desiredSpeeds = new ChassisSpeeds(vx, vy, omega).toRobotRelative(Rotation2d.fromDegrees(robotAngle));
    }

    public Rotation2d getYaw() {
        return imu.getRotation2d();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    @Override
    public void periodic() {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);

        poseEstimator.update(getYaw(), getModulePositions());

        field.setRobotPose(poseEstimator.getEstimatedPosition());

        SmartDashboard.putData("Field", field);
    }
}
