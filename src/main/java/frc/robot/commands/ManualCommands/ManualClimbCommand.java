package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ManualClimbCommand extends Command {
    ClimbSubsystem climbMotor;
    Supplier<Double> climbSpeed;

    public ManualClimbCommand(ClimbSubsystem climbMotor, Supplier<Double> climbSpeed){
        this.climbMotor = climbMotor;
        this.climbSpeed = climbSpeed;

        addRequirements(climbMotor);
    }

    @Override
    public void initialize(){
        System.out.println("Manual Climb Ready");
    }

    @Override
    public void execute(){
        climbMotor.runClimber(climbSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Manual Climber end interrupted:" + isInterrupted);
        climbMotor.runClimber(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}