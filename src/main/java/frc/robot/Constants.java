// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {   

    public final class USB{
        public static final int DRIVER_CONTROLLER = 0;      // Driver Controller USB ID
        public static final int OPERATOR_CONTROLLER = 1;    // Operator controller USB ID
        public static final int OPERATOR_LY = 1;
        public static final int OPERATOR_LX = 0;
        public static final int OPERATOR_RY = 5;
        public static final int OPERATOR_RX = 4;
        public static final int OPERATOR_RT = 3;
        public static final int OPERATOR_LT = 2;
    }

    public final class ModuleConstants{
        public static final double kWheelDiameterMeters = 0.1143; // 4.5 inches
        // gear ratio from thrifty swerve https://thethriftybot.com/products/thrify-swerve gear ratio options (pinion size 12 + second stage gear 16t? (only confirmed pinion))
        public static final double kDriveMotorGearRatio = 1/6.0;
        public static final double kTurningMotorGearRatio = 1 / 21.42857142857143;//(12*14)/(72*50) based on #of teeth
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; // Math.PI * kWheelDiameterMeters = Circumference
        public static final double kTurnEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRPM2RadPerSec = kTurnEncoderRot2Rad / 60;
        public static final double kPTurning = 0.2;
        public static final double kPDriving = 0.25;
        
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(28-2.5); // 28 width (motor center 2.5 inches from edge)
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(26-2.5); // 26 length
        // Distance between front and back wheels

        public static final double kRobotRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

        //TODO: Update to positive left positive forward FL, Fr, BL, BR
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 20;
        public static final int kFrontLeftTurningMotorPort = 21;

        public static final int kBackLeftDriveMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 11;

        public static final int kFrontRightDriveMotorPort = 26;
        public static final int kFrontRightTurningMotorPort = 25;

        public static final int kBackRightDriveMotorPort = 16;
        public static final int kBackRightTurningMotorPort = 15;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveReversed = false;
        public static final boolean kBackLeftDriveReversed = false;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kBackRightDriveReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 22;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 27;
        public static final int kBackRightDriveAbsoluteEncoderPort = 17;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // OLD OFFSETS
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRot = 0.849;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRot = 0.597;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRot = 0.148;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRot = 0.613;

        // OFFSETS
        // If robot is positively off, subtract
        // Else, add
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegree = 61.645715;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetDegree = 212.642091;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetDegree = 163.585028;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetDegree = 89.405267;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegree = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDegree = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDegree = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetDegree = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.3;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSlowButtonDriveModifier = 0.15;
        public static final double kSlowButtonTurnModifier = 0.30;
        public static final double teleSpeed = 0.45;
        public static final double teleTurnSpeed = 0.3;

        public static final double kPDrive = 1.5;
        public static final double kIDrive = 0.01; // Test again after robot gets wired, was 0.06 before test

        public static final double kPTurning = 5;
        public static final double kITurning = 0.05;



        public static final double kPDrift = 0.045;
        public static final double kIDrift = 0.0065; //Changed before test
        
        public static final double kPAlignTrench = 0.06;
        public static final double kIAlignTrench = 0.0;
    }

    public static final class TurretConstants {
        public static final int flywheelMotorLeaderID = 1;
        public static final int flywheelMotorFollowerID = 2;
        public static final boolean flywheelMotorReversed = false;
        public static final double kSfly = 0;
        public static final double kPfly = 0;
        public static final double kIfly = 0; 

        public static final int hoodMotorID = 10;
        public static final boolean hoodMotorReversed = true;
        public static final int hoodAbsoluteEncoderID = 11;
        public static final double kPhood = 0;
        // public static final double kIhood = 0; 

        public static final int turretMotorID = 20;
        public static final boolean turretMotorReversed = false;
        public static final int turretLimitSwitchID = 21;
        public static final double kPturret = 0;
        // public static final double kIhood = 0; 
    }

    public static final class IntakeArmConstants { // Update Id's and limits
        public static final double leftMotorID = 0;
        public static final double rightMotorID = 1;
        
        public static final double armExtendLimit = 152.0; 
    }

    public static final class IntakeArmStates {
        public static final double NONE = -1;
        public static final double START = 0;
        public static final double RAMP = 0;
        public static final double PICKUP = 1;
    }

    
    public static final class ShooterStates {
        public static final double NONE = 0;
        public static final double SHOOTING = 1;
    };
 
    public static final class hoodStates {
        public static final double NONE = 0;
        public static final double FIRST = 1;
    };

    public static final class TurretStates {
        public static final double NONE = 0;
        public static final double TRENCH = 90;
    };

    public static final class IntakeConstants {
        public static final int leftMotorID = 45;
        public static final int rightMotorID = 44;
        public static final int rollerMotorID = 46;

        public static final int intakeArmZeroLimitSwitchID = 1;

        public static final double rollerkP = 0.01;
        public static final double armkP = 0.01;
    }

    public static final class IntakeRollerStates {
        public static final double NONE = 0;
        public static final double IN = 1;
        public static final double OUT = 2;
    };

    public static final class IndexerConstants {
        public static final int indexerMotorID = 3;
        public static final int kickerMotorID = 5;
    }

    public static final class ClimbConstants {
        public static final int climbMotorID = 61;

        public static final int kP = 0;
        public static final int kI = 0;
    }

    public static final class ClimbStates {
        public static final double NONE = -1;
        public static final double RETRACTED = 0;
        public static final double L1 = 1;
    }

    public static final class LEDConstants {
        public static final int numLEDsPerStrip = 24;
        public static final int numLEDsPerColor = 3;
    }

    public static final class GameConstants {
        public static final int Robot = 0;
        public static final int Auto = 1;
        public static final int TeleOp = 2;
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        // public static final double kPXController = 5;
        // public static final double kPYController = 5;
        // public static final double kPThetaController = 2.5;

        // public static final double kIXController = 0.35; //0.06
        // public static final double kIYController = 0.35; // 0.06
        // public static final double kIThetaController = 0.2;
        public static final double kPXController = 4.8;// 1.5
        public static final double kPYController = 4.8; // 1.5
        public static final double kPThetaController = 2.5; // 2.5

        public static final double kIXController = 0.01; //0.01 
        public static final double kIYController = 0.01; // 0.01
        public static final double kIThetaController = 0.01;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final double firstWait = 0.4;
        public static final double secondWait = 0.9;
        public static final double stationWait = 0.4;
    }

    public static final class XBoxConstants {
        
        // Buttons
        public static final int A = 1; 
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;

        public static final int LB = 5;
        public static final int R1 = 6;
        
        public static final int PAGE = 7;
        public static final int MENU = 8;

        public static final int L3 = 9;
        public static final int R3 = 10;

        // Axes

        public static final int LX = 0;
        public static final int LY = 1; 

        public static final int LT = 2;
        public static final int RT = 3;

        public static final int RX = 4; 
        public static final int RY = 5;

    }
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final int kDriverRB = 6;

        public static final double kDeadband = 0.05;

        //BUTTONS

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int L3 = 9;
        public static final int R3 = 10;

        //AXES

        public static final int LX = 0;
        public static final int LY = 1;
        public static final int LT = 2;

        public static final int RT = 3;
        public static final int RX = 4;
        public static final int RY = 5;
    }

    public static final class LimelightConstants {
        public static final String turretName = "limelight";
        public static final String gamePieceName = "limelight-gp";
        public static final String climbName = "limelight-climb";
        public static final int Estimate_Distance = 20;
        public static final int aprilTagPipeline = 0;
        public static final int gamePiecePipeline = 1;
    }

    public static final class Colors {
        public static final Color red = Color.kRed;
        public static final Color blue = Color.kBlue;
        public static final Color green = new Color(0, 255, 0);
        public static final Color white = Color.kWhite;
        public static final Color uRed = new Color(20, 0 , 0);
        public static final Color uDarkOrange = new Color(254,17,1);
        public static final Color uGreen = new Color(0, 7, 0);
        public static final Color uOrange = new Color(255, 25, 0);
        public static final Color uGold = Color.kGold; //new Color(255, 215, 0);
        public static final Color REEFSCAPE_COLOR = Color.kAliceBlue;
        public static final Color[] uColors = {uRed, uDarkOrange, uGreen, uOrange};
    }

    public static final class Measurements {
        // all in m
        // robot l/w 28.5 by 28.5 inches
        // bumper width ~= 3 inches
        //** */ TODO: Change this to 2026 game //** */
        public static final double robotCenterToFront = 0.45085 + 0.03 + 0.2; // robot length/2 + bumper width = 14.25 + 3.5 = 17.75 inches  
        // public static final double robotCenterToFront = 0.50085; // Test
        public static final double robotSideOffset = 0.0254; // intake is 1 inch to the left so move robot 1 inch to the right
        public static final double branchOffset = 0.1651; // 6.5 inches
        public static final double coralStationDivotOffset = 0.2032; // 8 inches
    }

    public static final PathConstraints pathConstraints = new PathConstraints(4.8, 1.3, 540, 720);
}
