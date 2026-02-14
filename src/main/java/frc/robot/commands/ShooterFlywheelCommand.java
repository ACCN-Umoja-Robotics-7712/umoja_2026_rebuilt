package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShooterFlywheelCommand extends Command {
    ShooterFlywheelSubsystem flywheelMotorLeader;
    Supplier<Double> shooterSpeedSupplier;

    public ShooterFlywheelCommand(ShooterFlywheelSubsystem flywheelMotorLeader, Supplier<Double> shooterSpeedSupplier){
        this.flywheelMotorLeader = flywheelMotorLeader;
        this.shooterSpeedSupplier = shooterSpeedSupplier;
        addRequirements(flywheelMotorLeader); 
    }

    @Override
    public void initialize(){
        System.out.println("Flywheel Motor Initialized");
    }

    @Override
    public void execute(){
        flywheelMotorLeader.runShooter(shooterSpeedSupplier.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Shooter is interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}