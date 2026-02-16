// This is the subsystem for the indexer and kicker

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private SparkFlex indexerMotor;
    private SparkFlex kickerMotor;
    
    public IndexerSubsystem() {
        indexerMotor = new SparkFlex(IndexerConstants.indexerMotorID, MotorType.kBrushless);
        kickerMotor = new SparkFlex(IndexerConstants.kickerMotorID, MotorType.kBrushless);
    }

    public void runIndexer(double speed) { // Can change the speed for each motor independently
        indexerMotor.set(speed);
        kickerMotor.set(speed*0.3);
    }

}
