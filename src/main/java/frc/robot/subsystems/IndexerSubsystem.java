// This is the subsystem for the indexer and kicker

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private SparkFlex indexerMotorOriginal;
    private SparkFlex beltMotor;
    
    public IndexerSubsystem() {
        indexerMotorOriginal = new SparkFlex(IndexerConstants.indexerMotorLeaderID, MotorType.kBrushless);
        SparkBaseConfig indexerOriginalConfig = new SparkFlexConfig().smartCurrentLimit(55, 20); // NEO_Vortex (80 A but was 40 A)
        indexerMotorOriginal.configure(indexerOriginalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        beltMotor = new SparkFlex(IndexerConstants.beltMotorID, MotorType.kBrushless);
        SparkBaseConfig beltConfig = new SparkFlexConfig().smartCurrentLimit(35, 20); // NEO_Vortex
        beltConfig.inverted(true);
        beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIndexerAtVoltage(double indexVoltage, double beltVoltage) { // Can change the speed for each motor independently
        indexerMotorOriginal.setVoltage(indexVoltage);
        beltMotor.setVoltage(beltVoltage); // 3.0
    }

    public void runIndexerUsingTestvoltage() {
        double indexVoltage = SmartDashboard.getNumber("TEST Indexer with applied voltage", 10);
        double indexBeltVoltage = SmartDashboard.getNumber("TEST Indexer belt with applied voltage", 10/2.25);
        indexerMotorOriginal.setVoltage(indexVoltage);
        beltMotor.setVoltage(indexBeltVoltage);

    }
    
    public void stopIndexer() {
        indexerMotorOriginal.stopMotor();
        beltMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer velocity", indexerMotorOriginal.getEncoder().getVelocity());
        SmartDashboard.putNumber("Indexer Belt velocity", beltMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Indexer Belt current", beltMotor.getOutputCurrent());


        double indexVoltage = SmartDashboard.getNumber("TEST Indexer with applied voltage", 10);
        double indexBeltVoltage = SmartDashboard.getNumber("TEST Indexer belt with applied voltage", 10/2.25);
        SmartDashboard.putNumber("TEST Indexer with applied voltage", indexVoltage);
        SmartDashboard.putNumber("TEST Indexer belt with applied voltage", indexBeltVoltage);
    }

}
