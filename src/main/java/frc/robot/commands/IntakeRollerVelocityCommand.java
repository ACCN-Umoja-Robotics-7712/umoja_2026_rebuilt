package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class IntakeRollerVelocityCommand extends Command {
    IntakeRollerSubsystem intakeSubsystem;
    Supplier<Double> intakeRPMSupplier;

    public IntakeRollerVelocityCommand(IntakeRollerSubsystem intakeSubsystem, Supplier<Double> intakeRPMSupplier){
        this.intakeSubsystem = intakeSubsystem;
        this.intakeRPMSupplier = intakeRPMSupplier;
        addRequirements(intakeSubsystem); 
    }

    @Override
    public void initialize(){
        System.out.println("Intake RPM Initialized");
    }

    @Override
    public void execute(){
        intakeSubsystem.setIntakeVelocity(intakeRPMSupplier.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Intake RPM INTERRUPTED:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}