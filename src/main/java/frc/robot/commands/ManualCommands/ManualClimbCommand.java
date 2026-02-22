package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ManualClimbCommand extends Command {
    ClimbSubsystem climbSubsystem;
    Supplier<Double> climbSpeed;

    public ManualClimbCommand(ClimbSubsystem climbSubsystem, Supplier<Double> climbSpeed){
        this.climbSubsystem = climbSubsystem;
        this.climbSpeed = climbSpeed;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("Manual Climb Ready");
    }

    @Override
    public void execute(){
        climbSubsystem.runClimber(climbSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Climber end interrupted:" + isInterrupted);
        climbSubsystem.runClimber(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}