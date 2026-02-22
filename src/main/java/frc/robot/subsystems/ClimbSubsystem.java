package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbStates;

public class ClimbSubsystem extends SubsystemBase {

    SparkMax climbMotor;
    private double state = Constants.ClimbStates.NONE; 
    PIDController climbPID = new PIDController(ClimbConstants.kP, ClimbConstants.kI, 0);
    private final DigitalInput climbLimitSwitch = new DigitalInput(2);
    
    public ClimbSubsystem(){

        SparkBaseConfig climbConfig = new SparkFlexConfig().smartCurrentLimit(15); // Neo_550: Current Limit is 15
        climbConfig.idleMode(IdleMode.kBrake);
        climbConfig.inverted(false);

        climbMotor = new SparkMax(Constants.ClimbConstants.climbMotorID, MotorType.kBrushless);

        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        
    }
    
    public void runClimber(double percent){
        if(!climbLimitSwitch.get()){ //If the climber isn't fully down operate regularly
            climbMotor.set(percent);
        } 
        else { //If the climber is down:
            if(percent<0){ //If they want to move the climber up, allow it.
                climbMotor.set(percent);
            }
            else { //Don't let them bring the climber back down
                climbMotor.set(0);
            }
        } 
    }

    public void setClimbPosition(double wantedPosition) {
        climbMotor.set(climbPID.calculate(climbMotor.getEncoder().getPosition(), wantedPosition));
    }

    public void zeroClimbPosition() {
        climbMotor.getEncoder().setPosition(0);
    }

    public double getState(){
        return state;
    }

    public void setState(double state){
        if (this.state != state) {
            this.state = state;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run (Change to Elastic - Lewi)
        SmartDashboard.putNumber("Climber position", climbMotor.getEncoder().getPosition());

        if (state == ClimbStates.NONE) {
        } else {
            setClimbPosition(state);
        }
    }
}