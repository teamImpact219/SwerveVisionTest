package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem for AprilTag tracking using PhotonVision.
 * Processes camera frames to track a specific AprilTag and provides
 * target visibility and yaw angle for aiming.
 */
public class Vision extends SubsystemBase {
    private final PhotonCamera camera;

    private boolean targetVisible = false;
    private double targetYaw = 0.0;
    private int trackedTagId = -1;

    /**
     * Creates a new Vision subsystem.
     *
     * @param cameraName The name of the PhotonVision camera as configured in the PhotonVision UI.
     */
    public Vision(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    /**
     * Processes the latest camera frames and updates target tracking state.
     * Uses getAllUnreadResults() to ensure only fresh frames are processed.
     */
    @Override
    public void periodic() {
        targetVisible = false;

        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == trackedTagId) {
                        targetYaw = target.getYaw();
                        targetVisible = true;
                        break;
                    }
                }
            }
        }

        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        SmartDashboard.putNumber("Vision Target Yaw", targetYaw);
    }

    /**
     * Sets the AprilTag ID to track.
     *
     * @param tagId The fiducial ID of the AprilTag to track, or -1 to disable tracking.
     */
    public void setTrackedTagId(int tagId) {
        this.trackedTagId = tagId;
    }

    /**
     * Returns whether the tracked AprilTag is currently visible.
     *
     * @return true if the tracked tag is visible, false otherwise.
     */
    public boolean hasTarget() {
        return targetVisible;
    }

    /**
     * Returns the yaw angle to the tracked AprilTag.
     *
     * @return The yaw angle in degrees. Positive values indicate the target is to the right.
     */
    public double getTargetYaw() {
        return targetYaw;
    }

    /**
     * Returns the underlying PhotonCamera for advanced usage.
     *
     * @return The PhotonCamera instance.
     */
    public PhotonCamera getCamera() {
        return camera;
    }
}
