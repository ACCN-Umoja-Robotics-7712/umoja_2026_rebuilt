package frc.robot.commands;

import java.util.function.Supplier;
?
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class ShooterHoodCommand extends Command{
    ShooterHoodSubsystem ShooterHood;
    Supplier<Double> ShooterHoodSpeed;

    public ShooterHoodCommand(ShooterHoodSubsystem ShooterHood, Supplier<Double> ShooterHoodSpeed){
        this.ShooterHood = ShooterHood;
        this.ShooterHoodSpeed = ShooterHoodSpeed;

        addRequirements(ShooterHood);
    }

    @Override
    public void initialize(){
        System.out.println("Manual Shooter Hood Initialized");
    }

    @Override
    public void execute(){
        ShooterHood.runShooterHood(ShooterHoodSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Shooter Hood end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    