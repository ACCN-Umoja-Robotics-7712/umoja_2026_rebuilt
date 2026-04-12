package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class IndexerVelocityCommand extends Command {
    IndexerSubsystem indexerSubsystem;
    Supplier<Double> indexerRPMSupplier;

    public IndexerVelocityCommand(IndexerSubsystem indexerSubsystem, Supplier<Double> indexerRPMSupplier){
        this.indexerSubsystem = indexerSubsystem;
        this.indexerRPMSupplier = indexerRPMSupplier;
        addRequirements(indexerSubsystem); 
    }

    @Override
    public void initialize(){
        System.out.println("Indexer RPM Initialized");
    }

    @Override
    public void execute(){
        indexerSubsystem.setIndexerVelocity(indexerRPMSupplier.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Indexer RPM INTERRUPTED:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}