package frc.robot.commands.ZeroCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class zeroIntakeArm extends Command {
    
    IntakeArmSubsystem intakeArmSubsystem;

    public zeroIntakeArm(IntakeArmSubsystem intakeArmSubsystem){
        this.intakeArmSubsystem = intakeArmSubsystem;
    }

    @Override
    public void initialize(){
        System.out.println("Intake Arm zero'd");
    }

    @Override
    public void execute(){
        this.intakeArmSubsystem.zeroIntakeArm();
    }

    @Override
    public void end(boolean isInterrupted){

    }
};
