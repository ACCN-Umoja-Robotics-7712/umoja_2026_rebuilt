package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class StopMotorCommand extends InstantCommand {
    ShooterFlywheelSubsystem flywheelMotorSubsystem;
    Supplier<Double> shooterRPMSupplier;

    public StopMotorCommand(ShooterFlywheelSubsystem flywheelMotorSubsystem, Supplier<Double> shooterRPMSupplier){
        this.flywheelMotorSubsystem = flywheelMotorSubsystem;
        this.shooterRPMSupplier = shooterRPMSupplier;
        addRequirements(flywheelMotorSubsystem); 
    }

    @Override
    public void initialize(){
        System.out.println("Flywheel Motor Stopped");
        flywheelMotorSubsystem.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}