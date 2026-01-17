package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    ShooterSubsystem shooterSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Shoot Command Started");
    }

    @Override
    public void execute () {
        shooterSubsystem.runShooter(0.5);
    }

    @Override
    public void end(boolean isInterrupted) {
        System.out.println("Align with tag end is interrupted" + isInterrupted);
        shooterSubsystem.runShooter(0);
    }
}
