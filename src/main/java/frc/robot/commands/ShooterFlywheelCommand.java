package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShooterFlywheelCommand extends Command{
    ShooterFlywheelSubsystem shooterFlywheel;
    Supplier<Double> flywheelSpeed;

    public void ManualShooterFlywheel(ShooterFlywheelSubsystem shooterFlywheel, Supplier<Double> flywheelSpeed){
        this.shooterFlywheel = shooterFlywheel;
        this.flywheelSpeed = flywheelSpeed;
        addRequirements(shooterFlywheel);
    }

    @Override
    public void initialize(){
        System.out.println("Shooter flywheel Initialized");
    }

    @Override
    public void execute(){
        shooterFlywheel.runShooter(flywheelSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Shooter flywheel end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
