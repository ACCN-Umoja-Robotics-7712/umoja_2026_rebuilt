package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

/**
 * Tracks shooter readiness across flywheel, turret, and hood subsystems.
 *
 * <p>Uses hysteresis (a minimum dwell timer) to prevent rapid on/off jitter when
 * the system is right at the edge of its tolerance band. All three subsystems
 * must be within tolerance for at least {@link #kMinStableSeconds} before
 * {@link #isReady()} returns {@code true}.
 *
 * <p>Pattern adapted from Team 6328 Mechanical Advantage's state-machine approach:
 * sample a stable "ready" state over time rather than using a raw instantaneous
 * boolean every 20 ms loop.
 */
public class ShooterReadinessTracker {

    /**
     * Minimum time (seconds) that ALL three systems must be within tolerance
     * before {@link #isReady()} reports {@code true}. Increase this if the
     * indexer is firing too early; decrease it to improve cycle time.
     */
    public static final double kMinStableSeconds = 0.1;

    private final ShooterFlywheelSubsystem flywheel;
    private final ShooterTurretSubsystem turret;
    private final ShooterHoodSubsystem hood;

    private final Timer stableTimer = new Timer();
    private boolean wasReady = false;

    /**
     * @param flywheel Flywheel subsystem.
     * @param turret   Turret subsystem.
     * @param hood     Hood subsystem.
     */
    public ShooterReadinessTracker(
            ShooterFlywheelSubsystem flywheel,
            ShooterTurretSubsystem turret,
            ShooterHoodSubsystem hood) {
        this.flywheel = flywheel;
        this.turret   = turret;
        this.hood     = hood;
        stableTimer.start();
    }

    /**
     * Returns {@code true} only when flywheel, turret, AND hood have all been
     * within their respective tolerances for at least {@link #kMinStableSeconds}.
     *
     * <p>Call this every scheduler loop (20 ms) from a command's {@code execute()}.
     */
    public boolean isReady() {
        boolean flywheelReady = flywheel.didReachVelocity();
        boolean turretReady   = turret.didReachAngle();
        boolean hoodReady     = hood.didReachValue();
        boolean allInTolerance = flywheelReady && turretReady && hoodReady;

        if (!allInTolerance) {
            // Any system left tolerance — reset dwell timer and clear the ready flag.
            stableTimer.reset();
            wasReady = false;
        } else if (!wasReady) {
            // All systems in tolerance; promote to "ready" once dwell time has elapsed.
            wasReady = stableTimer.get() >= kMinStableSeconds;
        }
        // Once wasReady is true it stays true until a system leaves tolerance.

        SmartDashboard.putBoolean("Tracker/Flywheel Ready", flywheelReady);
        SmartDashboard.putBoolean("Tracker/Turret Ready",   turretReady);
        SmartDashboard.putBoolean("Tracker/Hood Ready",     hoodReady);
        SmartDashboard.putBoolean("Tracker/All Ready",      wasReady);
        SmartDashboard.putString("Tracker/Not Ready Reason",
                getNotReadyReason(flywheelReady, turretReady, hoodReady));

        return wasReady;
    }

    /** Human-readable explanation of which system is still not ready. */
    private String getNotReadyReason(boolean fw, boolean tur, boolean hd) {
        if (fw && tur && hd) {
            return "ALL READY";
        }
        StringBuilder sb = new StringBuilder();
        if (!fw)  sb.append("Flywheel ");
        if (!tur) sb.append("Turret ");
        if (!hd)  sb.append("Hood ");
        sb.append("not ready");
        return sb.toString();
    }
}
