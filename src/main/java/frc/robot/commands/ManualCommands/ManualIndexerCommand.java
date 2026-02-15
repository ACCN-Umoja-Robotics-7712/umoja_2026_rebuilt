package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class ManualIndexerCommand extends Command {
    IndexerSubsystem indexerMotor;
    IndexerSubsystem kickerMotor;

    Supplier<Double> indexerMotorSpedSupplier;

    public ManualIndexerCommand(IndexerSubsystem indexerMotor, IndexerSubsystem kickerMotor, Supplier<Double> indexerMotorSupplier){
        this.indexerMotor = indexerMotor;
        this.kickerMotor = kickerMotor;

        addRequirements();
    }

}
