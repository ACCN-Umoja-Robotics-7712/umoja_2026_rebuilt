package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShootCommand extends Command {
    private ShooterFlywheelSubsystem flywheel;
    private double flywheelState;

    public ShootCommand(ShooterFlywheelSubsystem flywheel, double flywheelState) {
        this.flywheel = flywheel;
        this.flywheelState = flywheelState;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.setState(flywheelState);
        System.out.println("Shoot Command Started");
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isInterrupted) {
        System.out.println("SHOOTER END INTERRUPTED: " + isInterrupted);
        // BUG FIX: flywheel motor was left spinning at its last commanded speed when
        // the command ended. stopFlywheel() immediately commands 0 V.
        flywheel.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return flywheel.didReachVelocity();
    }
}
