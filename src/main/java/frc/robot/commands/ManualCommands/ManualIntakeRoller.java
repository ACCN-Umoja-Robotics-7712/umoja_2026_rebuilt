package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class ManualIntakeRoller extends Command{
    IntakeRollerSubsystem intakeRoller;
    Supplier<Double> rollerSpeed;

    public ManualIntakeRoller(IntakeRollerSubsystem intakeRoller, Supplier<Double> rollerSpeed){
        this.intakeRoller = intakeRoller;
        this.rollerSpeed = rollerSpeed;
        addRequirements(intakeRoller);
    }

    @Override
    public void initialize(){
        System.out.println("Intake Rollers Initialized");
    }

    @Override
    public void execute(){
        intakeRoller.runIntake(rollerSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Intake Rollers end interrupted:" + isInterrupted);
        intakeRoller.runIntake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
