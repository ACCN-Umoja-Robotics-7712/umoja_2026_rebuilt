
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class MoveIntakeArm extends Command{
    IntakeArmSubsystem arm;
    double armState;

    public MoveIntakeArm(IntakeArmSubsystem arm, double armState){
        this.arm = arm;
        this.armState = armState;

        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.setState(armState);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("ARM END");
    }

    @Override
    public boolean isFinished() {
        return arm.didReachState();
    }
}