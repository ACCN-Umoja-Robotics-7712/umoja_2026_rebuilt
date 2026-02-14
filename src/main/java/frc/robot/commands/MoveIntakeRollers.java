
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class MoveIntakeRollers extends Command{
    IntakeRollerSubsystem roller;
    double rollerState;

    public MoveIntakeRollers(IntakeRollerSubsystem roller, double rollerState){
        this.roller = roller;
        this.rollerState = rollerState;

        addRequirements(roller);
    }

    @Override
    public void initialize(){
        roller.setState(rollerState);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("ROLLER END");
    }

    @Override
    public boolean isFinished() {
        return roller.didReachState();
    }
}