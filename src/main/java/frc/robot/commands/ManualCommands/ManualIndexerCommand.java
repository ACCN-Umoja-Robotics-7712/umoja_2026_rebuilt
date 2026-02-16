package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class ManualIndexerCommand extends Command {
    IndexerSubsystem indexerSubsystem;
    Supplier<Double> indexerMotorSpeedSupplier;

    public ManualIndexerCommand(IndexerSubsystem indexerSubsystem, Supplier<Double> indexerMotorSpeedSupplier){
        this.indexerSubsystem = indexerSubsystem;
        this.indexerMotorSpeedSupplier = indexerMotorSpeedSupplier;

        addRequirements(indexerSubsystem);
    }  

    @Override
    public void initialize(){
        System.out.println("Manual Indexer Initialized");
    }

    @Override
    public void execute(){
        indexerSubsystem.runIndexer(indexerMotorSpeedSupplier.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Indexer end interrupted:" + isInterrupted);
        indexerSubsystem.runIndexer(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
