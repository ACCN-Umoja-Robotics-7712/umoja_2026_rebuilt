package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterStates;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class ShooterHoodValueCommand extends Command {
    ShooterHoodSubsystem hood;
    Supplier<Double> wantedHoodValue;

    public ShooterHoodValueCommand(ShooterHoodSubsystem hood, Supplier<Double> wantedHoodValue){
        this.hood = hood;
        this.wantedHoodValue = wantedHoodValue;

        addRequirements(hood); 
    }

    @Override
    public void initialize(){
        System.out.println("Hood Initialized");
    }

    @Override
    public void execute(){
        hood.setHoodValue(wantedHoodValue.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Hood end interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
    