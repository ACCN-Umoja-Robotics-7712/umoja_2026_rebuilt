package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShootCommand extends Command {
    private ShooterFlywheelSubsystem flywheel;

    public ShootCommand(ShooterFlywheelSubsystem flywheel) {
        this.flywheel = flywheel;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        System.out.println("Shoot Command Started");
    }

    @Override
    public void execute() {
        flywheel.runShooter(0.5);
    }

    // TODO: Turret turning and hood control

    @Override
    public void end(boolean isInterrupted) {
        System.out.println("Shoot Command ended, interrupted: " + isInterrupted);
        flywheel.runShooter(0);
    }
}
