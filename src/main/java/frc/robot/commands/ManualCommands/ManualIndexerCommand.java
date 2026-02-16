package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class ManualIndexerCommand extends Command {
    IndexerSubsystem indexerMotor;
    Supplier<Double> indexerMotorSpeedSupplier;

    public ManualIndexerCommand(IndexerSubsystem indexerMotor, Supplier<Double> indexerMotorSpeedSupplier){
        this.indexerMotor = indexerMotor;
        this.indexerMotorSpeedSupplier = indexerMotorSpeedSupplier;

        addRequirements(indexerMotor);
    }  

    @Override
    public void initialize(){
        System.out.println("Manual Indexer Initialized");
    }

    @Override
    public void execute(){
        indexerMotor.runIndexer(indexerMotorSpeedSupplier.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Indexer end interrupted:" + isInterrupted);
        indexerMotor.runIndexer(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
