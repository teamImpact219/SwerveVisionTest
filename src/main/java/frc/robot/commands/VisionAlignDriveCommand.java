package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * A teleop drive command with optional vision-assisted rotation alignment.
 *
 * This command provides standard field-centric swerve drive control while allowing
 * the driver to hold a button to automatically align the robot's heading toward a
 * vision target (e.g., an AprilTag). Translation remains under full manual control.
 *
 * Usage:
 * Set as the drivetrain's default command. The driver controls translation normally.
 * When the align button is held and a target is visible, the robot automatically
 * rotates to center the target in the camera's field of view.
 *
 * Tuning:
 * Adjust VISION_TURN_kP to change alignment responsiveness:
 *   - Too low: Sluggish alignment, may not reach target
 *   - Too high: Oscillation or overshoot around target
 */
public class VisionAlignDriveCommand extends Command {

    // === Subsystems ===
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // === Drive configuration ===
    private final SwerveRequest.FieldCentric driveRequest;
    private final double maxSpeed;        // Maximum translational speed (m/s)
    private final double maxAngularRate;  // Maximum rotational rate (rad/s)

    // === Input suppliers (from joysticks/buttons) ===
    private final DoubleSupplier xSupplier;    // Forward/backward (-1 to 1)
    private final DoubleSupplier ySupplier;    // Left/right strafe (-1 to 1)
    private final DoubleSupplier rotSupplier;  // Rotation (-1 to 1)
    private final BooleanSupplier alignButton; // Vision align trigger

    /**
     * Proportional gain for vision alignment.
     * Multiplied by target yaw (degrees) to produce a turn rate.
     * Start low (0.05-0.1) and increase until alignment is responsive but stable.
     */
    private static final double VISION_TURN_kP = 0.1;

    /**
     * Creates a new VisionAlignDriveCommand.
     *
     * @param drivetrain     The swerve drivetrain subsystem
     * @param vision         The vision subsystem for target detection
     * @param driveRequest   A field-centric swerve request to configure and reuse
     * @param xSupplier      Supplier for forward/backward input (-1 to 1)
     * @param ySupplier      Supplier for left/right strafe input (-1 to 1)
     * @param rotSupplier    Supplier for rotation input (-1 to 1)
     * @param alignButton    Supplier that returns true when vision align is requested
     * @param maxSpeed       Maximum translational velocity in meters per second
     * @param maxAngularRate Maximum rotational velocity in radians per second
     */
    public VisionAlignDriveCommand(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            SwerveRequest.FieldCentric driveRequest,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier,
            BooleanSupplier alignButton,
            double maxSpeed,
            double maxAngularRate) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driveRequest = driveRequest;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.alignButton = alignButton;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        addRequirements(drivetrain);
        // Vision is read-only, not a requirement (allows other commands to use it)
    }

    /**
     * Called repeatedly while the command is scheduled.
     *
     * Reads joystick inputs and applies them to the drivetrain. When the align
     * button is held AND a vision target is visible, the rotation control is
     * overridden with proportional feedback to center the target in view.
     *
     * Translation (forward/strafe) remains under full driver control at all times.
     */
    @Override
    public void execute() {
        // Scale joystick inputs (-1 to 1) to actual velocities
        double forward = xSupplier.getAsDouble() * maxSpeed;       // m/s, field-relative X
        double strafe = ySupplier.getAsDouble() * maxSpeed;        // m/s, field-relative Y
        double turn = rotSupplier.getAsDouble() * maxAngularRate;  // rad/s, default to manual

        // Vision alignment: override rotation when button held and target visible
        if (alignButton.getAsBoolean() && vision.hasTarget()) {
            // P-controller: turn rate proportional to how far off-center the target is
            // Negative sign: positive yaw (target right of center) -> turn right (negative omega)
            turn = -1.0 * vision.getTargetYaw() * VISION_TURN_kP * maxAngularRate;
        }

        // Send velocities to the swerve drivetrain
        drivetrain.setControl(
            driveRequest.withVelocityX(forward)
                .withVelocityY(strafe)
                .withRotationalRate(turn)
        );
    }
}
