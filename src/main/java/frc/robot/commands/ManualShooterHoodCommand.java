package frc.robot.commands;

import java.util.function.Supplier;
?
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class ManualShooterHoodCommand extends Command{
    ManualShooterHoodSubsystem ManualShooterHood;
    Supplier<Double> ManualShooterHoodSpeed;

    public ManualShooterHoodCommand(ManualShooterHoodSubsystem ManualShooterHood, Supplier<Double> ManualShooterHoodSpeed){
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
        ManualShooterHood.runManualShooterHood(ManualShooterHoodSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Shooter Hood end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    