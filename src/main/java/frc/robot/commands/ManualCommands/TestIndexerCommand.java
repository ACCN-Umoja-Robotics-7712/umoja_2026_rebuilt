package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class TestIndexerCommand extends Command {
    IndexerSubsystem indexerSubsystem;
    Supplier<Double> indexerMotorVoltageSupplier;

    public TestIndexerCommand(IndexerSubsystem indexerSubsystem){
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(indexerSubsystem);
    }  

    @Override
    public void initialize(){
        System.out.println("TEST Indexer Initialized");
    }

    @Override
    public void execute(){
        indexerSubsystem.runIndexerUsingTestvoltage();
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("TEST Indexer end interrupted:" + isInterrupted);
        indexerSubsystem.runIndexerAtVoltage(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
