package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionSubsystem extends SubsystemBase {

    // TODO: Change to match the camera name configured in the PhotonVision UI
    private static final String CAMERA_NAME = "YOUR_CAMERA_NAME";

    // Transform from robot center to camera lens.
    // Positive X = forward, Y = left, Z = up (meters).
    // Rotation: roll, pitch, yaw (radians). Negative pitch = tilted downward.
    // TODO: Measure these values accurately on the physical robot.
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.3, 0.0, 0.3),          // 30 cm forward, 30 cm up
        new Rotation3d(0.0, Math.toRadians(-15), 0.0) // 15 degrees tilted down
    );

    // Vision measurement trust levels [x (m), y (m), heading (rad)].
    // Higher values = less trust in that measurement.
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 1.0);

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        camera = new PhotonCamera(CAMERA_NAME);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            ROBOT_TO_CAMERA
        );
    }

    @Override
    public void periodic() {
        for (var result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) continue;

            poseEstimator.update(result).ifPresent(estimated -> {
                drivetrain.addVisionMeasurement(
                    estimated.estimatedPose.toPose2d(),
                    estimated.timestampSeconds,
                    getStdDevs(estimated)
                );
            });
        }
    }

    /**
     * Scales measurement trust based on how many tags are visible and how far away they are.
     * Single-tag estimates beyond 4 m are rejected (extremely high std devs).
     */
    private Matrix<N3, N1> getStdDevs(EstimatedRobotPose estimated) {
        int numTags = estimated.targetsUsed.size();

        if (numTags >= 2) {
            return MULTI_TAG_STD_DEVS;
        }

        double avgDistMeters = estimated.targetsUsed.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);

        if (avgDistMeters > 4.0) {
            // Too far away for a reliable single-tag estimate; effectively reject it
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        // Scale std devs quadratically with distance so trust decreases at range
        return SINGLE_TAG_STD_DEVS.times(1.0 + avgDistMeters * avgDistMeters);
    }
}
