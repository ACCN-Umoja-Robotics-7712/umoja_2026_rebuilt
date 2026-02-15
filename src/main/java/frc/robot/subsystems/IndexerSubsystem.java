// This is the subsystem for the indexer and kicker

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class IndexerSubsystem {
    private TalonFX indexerMotor;
    private TalonFX kickerMotor;
    
    public IndexerSubsystem() {
        CANBus rio = new CANBus("rio");
        indexerMotor = new TalonFX(IndexerConstants.indexerMotorID, rio);
        kickerMotor = new TalonFX(IndexerConstants.kickerMotorID, rio);
    }

    public void runIndexer(double speed) { // Can change the speed for each motor independently
        indexerMotor.set(speed);
        kickerMotor.set(speed);
    }

}
