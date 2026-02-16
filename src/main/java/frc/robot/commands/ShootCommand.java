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

    // TODO: Turret turning and hood control

    @Override
    public void end(boolean isInterrupted) {
        System.out.println("Shoot Command ended, interrupted: " + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return flywheel.didReachState();
    }
}
