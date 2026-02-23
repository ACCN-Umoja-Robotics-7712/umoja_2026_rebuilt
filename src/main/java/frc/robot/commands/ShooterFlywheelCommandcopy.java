package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShooterFlywheelCommandcopy extends Command {
    ShooterFlywheelSubsystem flywheelMotorLeader;
    Supplier<Double> shooterRPMSupplier;

    public ShooterFlywheelCommandcopy(ShooterFlywheelSubsystem flywheelMotorLeader, Supplier<Double> shooterRPMSupplier){
        this.flywheelMotorLeader = flywheelMotorLeader;
        this.shooterRPMSupplier = shooterRPMSupplier;
        addRequirements(flywheelMotorLeader); 
    }

    @Override
    public void initialize(){
        System.out.println("Flywheel Motor Initialized");
    }

    @Override
    public void execute(){
        flywheelMotorLeader.setShooterVelocity(shooterRPMSupplier.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("SHOOTER END INTERRUPTED:" + isInterrupted);
        flywheelMotorLeader.runShooter(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}