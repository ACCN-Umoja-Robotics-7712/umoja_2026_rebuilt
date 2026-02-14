package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class ManualShooterHoodCommand extends Command{
    ShooterHoodSubsystem ManualShooterHood;
    Supplier<Double> ManualShooterHoodSpeed;

    public ManualShooterHoodCommand(ShooterHoodSubsystem ManualShooterHood, Supplier<Double> ManualShooterHoodSpeed){
        this.ManualShooterHood = ManualShooterHood;
        this.ManualShooterHoodSpeed = ManualShooterHoodSpeed;

        addRequirements(ManualShooterHood);
    }

    @Override
    public void initialize(){
        System.out.println("Manual Shooter Hood Initialized");
    }

    @Override
    public void execute(){
        ManualShooterHood.runHood(ManualShooterHoodSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Shooter Hood end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
    