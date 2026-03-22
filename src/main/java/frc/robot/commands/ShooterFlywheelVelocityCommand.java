package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShooterFlywheelVelocityCommand extends Command {
    ShooterFlywheelSubsystem flywheelMotorLeader;
    Supplier<Double> shooterRPMSupplier;

    public ShooterFlywheelVelocityCommand(ShooterFlywheelSubsystem flywheelMotorLeader, Supplier<Double> shooterRPMSupplier){
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
        // BUG FIX: motor was left at whatever voltage was last commanded when this
        // command ended (button release, timeout, or scheduler interruption).
        // stopFlywheel() calls SparkFlex.stopMotor() which commands 0 V immediately.
        flywheelMotorLeader.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}