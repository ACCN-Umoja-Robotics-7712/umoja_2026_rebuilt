// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
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

    public final class RobotConstants {
        public static final double robotWidth = 28;
        public static final double robotLength = 26;
        public static final double kRobotWeightKG = Units.lbsToKilograms(115);
        public static final double kBumperWeightKG = Units.lbsToKilograms(8);
        public static final double kBatteryWeightKG = Units.lbsToKilograms(12);
        public static final double kRobotTotalWeightKG = kRobotWeightKG + kBumperWeightKG + kBatteryWeightKG;
        public static final double kRobotMOI = 6.883; // kg*m^2, moment of inertia about the center of mass
    }

    public final class ModuleConstants{
        // For RobotConfig + robot setup
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.5); // 4.5 inches
        public static final double kWheelCOF = 1.200; // wheel COF inches
        public static final DCMotor kDriveMotor = DCMotor.getKrakenX60(1);
        public static final int kDriveMotorCurrentLimit = 60;
        public static final int kNumMotorsPerModule = 1; // number of drive motors per module (not including turning motors)

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
        public static final double kTrackWidth = Units.inchesToMeters(RobotConstants.robotWidth-2.5); // 28 width (motor center 2.5 inches from edge)
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(RobotConstants.robotLength-2.5); // 26 length
        // Distance between front and back wheels

        public static final double kRobotRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

        //TODO: Update to positive left positive forward FL, FR, BL, BR
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

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

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

        public static final double kSlowButtonDriveModifier = 0.5;
        public static final double kSlowButtonTurnModifier = 0.50;
        public static final double teleSpeed = 0.7;
        public static final double teleTurnSpeed = 0.55;
        
        public static final double shootingSpeedCap = 0.3; // 0.3m/s max speed when shooting, to improve accuracy

        public static final double kPDrive = 1.5;
        public static final double kIDrive = 0.01; // Test again after robot gets wired, was 0.06 before test

        public static final double kPTurning = 5;
        public static final double kITurning = 0.05;

        public static final double kPDrift = 0.06;
        public static final double kIDrift = 0.0065; //Changed before test
        
        public static final double kPAlignTrench = 0.06;
        public static final double kIAlignTrench = 0.0001;
    }

    public static final class TurretConstants {
        public static final int flywheelMotorLeaderID = 2;
        public static final int flywheelMotorFollowerID = 1;
        public static final boolean flywheelMotorReversed = false;
        public static final double kSfly = 0.13;
        public static final double kVfly = 0.0018;
        public static final double kPfly = 0.0001;
        public static final double kIfly = 0;
        public static final double kDfly = 0;
        public static final double kSkicker = 0;
        public static final double kVkicker = 0.0019;
        public static final double kPkicker = 0.00001;
        public static final double kIkicker = 0;

        public static final int hoodMotorID = 60;
        public static final boolean hoodMotorReversed = true;
        public static final int hoodAbsoluteEncoderID = 0;
        public static final double kPhood = 0.1;
        public static final double kShood = 0.2295;
        public static final double kIhood = 0.0; 
        public static final double kDhood = 0.0;

        public static final int turretMotorID = 55;
        public static final boolean turretMotorReversed = false;
        public static final int turretLimitSwitchID = 1;
        public static final double kPturretSlack = 0.1;
        public static final double kPturretSpring = 0;
        public static final double kIturretSlack = 0.02;
        public static final double kIturretSpring = 0;
        // public static final double kIhood = 0; 
        public static final double turretFakeFeedForward = 0.45;
        public static final double turretSpringResistance = 0.5;

        public static final int pitchOffset = 20; // 20 degrees up
        public static final int rollOffset = 0; // 0 degrees to the right
        public static final double upOffset = Units.inchesToMeters(18); // m up
        public static final double forwardOffset = 0; // m forward
        public static final double sideOffset = Units.inchesToMeters(6); // m to the right

        // public static final double turretMotorEncoderToRotationRatio = 1.0/42.0; // encoder is 42 ticks per rotation
        // motor -> motor to shaft (motor * shaft/motor * turret/shaft) = turret, 16:1 gear ratio from motor to shaft
        // motor -> shaft is 16:1, 16 motor rotations = 1 shaft rotation, and shaft -> turret is 125:35, 125 shaft rotation = 35 turret rotation, ~3.57:1
        public static final double motorToTurretRatio = (1.0/16.0) * (35.0/125.0); // motor rotations to turret rotations
        public static final double turretCenterToCameraCentreLength = Math.sqrt(forwardOffset * forwardOffset + sideOffset * sideOffset); // meters (Pythagorean theorem)
        public static final double turretCenterFromRobotCenterForwardLength = Units.inchesToMeters(-(RobotConstants.robotWidth/2) + 2 + (11.5/2)); // meters (negative cause turret is behind the robot center) 
        public static final double turretCenterFromRobotCenterSideLength = Units.inchesToMeters(-(RobotConstants.robotLength/2) + 2 + (11.5/2)); // meters (positive cause turret is to the left of the robot center)
    }

    public static final class IntakeArmStates {
        public static final double NONE = -1;
        public static final double START = 0;
        public static final double RAMP = 0;
        public static final double PICKUP = 1;
    }

    
    public static final class ShooterStates {
        public static final double NONE = 0;
        public static final double AUTO_BLUE = 1;
        public static final double AUTO_RED = 2;
    };

    public static final class IntakeConstants {
        public static final int leftMotorID = 45;
        public static final int rightMotorID = 44;
        public static final int rollerMotorID = 46;

        public static final int intakeArmZeroLimitSwitchID = 5;

        public static final double rollerkP = 0.01;
        public static final double armkP = 0.00;
        public static final double armkG = 0.00;
        public static final double armkV = 0.00;
    }

    public static final class IntakeRollerStates {
        public static final double NONE = 0;
        public static final double IN = 1;
        public static final double OUT = 2;
    };

    public static final class IndexerConstants {
        public static final int indexerMotorLeaderID = 3;
        public static final int kickerMotorID = 5;
        public static final int indexerMotorFollowerID = 34;
    }

    public static final class ClimbConstants {
        public static final int climbMotorID = 61; // Update it on the REV Hardware (Do it for all other motor IDs and Encoder IDs)
        public static final int climbLimitSwitchID = 2;
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
        public static final int Disabled = 0;
        public static final int Auto = 1;
        public static final int TeleOp = 2;
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        // public static final double kPXController = 5;
        // public static final double kPYController = 5;
        // public static final double kPThetaController = 2.5;

        // public static final double kIXController = 0.35; //0.06
        // public static final double kIYController = 0.35; // 0.06
        // public static final double kIThetaController = 0.2;
        public static final double kPXController = 4.8;// 1.5
        public static final double kPYController = 4.8; // 1.5
        public static final double kPThetaController = 1; // 2.5

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
        public static final String LIMELIGHT_LEFT = "limelight";
        public static final String LIMELIGHT_RIGHT = "limelight-four";
        public static final String LIMELIGHT_FORWARD = "limelight-driver";
        public static final int Estimate_Distance = 20;
        public static final int aprilTagPipeline = 0;
        public static final int gamePiecePipeline = 1;

        // Camera center about 19.5 inches up from the ground
        public static final double limelight2HeightOld = Units.inchesToMeters(19.5);
        // Camera center is from 8 inches from back + 1.5 inch forward
        public static final double limelight2ForwardOld = Units.inchesToMeters(-RobotConstants.robotLength/2 + 8 + 1.5);
        // Camera center is from right of robot + 2 * 1 inch in + 3.75 inch camera mount - 0.088583 to center of mounting hole - 2.835 distance between holes
        public static final double limelight2SideOld = Units.inchesToMeters((RobotConstants.robotWidth/2) - 2 - 3.75 + 0.088583 + 2.835);
    
        public static final double limelight2AngleOld = 30; // degrees, angle of the camera relative to horizontal, positive is looking up

        // new angles + locations for triple mount
        public static final double limelight4Height = Units.inchesToMeters(20.75);
        public static final double limelight4Forward = Units.inchesToMeters(-RobotConstants.robotLength/2 + 8);
        public static final double limelight4Side = Units.inchesToMeters((RobotConstants.robotWidth/2) - 1.5);
        public static final double limelight4Angle = 15; // degrees pitch

        public static final double limelight3Height = Units.inchesToMeters(20.75);
        public static final double limelight3Forward = Units.inchesToMeters(-RobotConstants.robotLength/2 + 8);
        // edge - bar - center ll - center current ll
        public static final double limelight3Side = Units.inchesToMeters((RobotConstants.robotWidth/2) - 2  - 5 - 1.4);
        public static final double limelight3Angle = 15; // degrees pitch

        public static final double limelight2Height = Units.inchesToMeters(20.5);
        public static final double limelight2Forward = Units.inchesToMeters(-RobotConstants.robotLength/2 + 8 + 0.5);
        public static final double limelight2Side = Units.inchesToMeters((RobotConstants.robotWidth/2) - 2 - 2.5);
        public static final double limelight2Angle = -10; // degrees pitch

        public static final int[] RED_HUB_CENTER_APRIL_TAG_IDS = new int[]{2,4,5,10};
        public static final int[] BLUE_HUB_CENTER_APRIL_TAG_IDS = new int[]{18,20,21,26};
        public static final int RED_HUB_FRONT_CENTER_APRIL_TAG_ID = 10;
        public static final int BLUE_HUB_FRONT_CENTER_APRIL_TAG_ID = 26;
        
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;
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

    public static final class SHOOTING_POSES {

        // BLUE SIDE
        public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.5, 4, new Rotation2d(0));
        public static final Pose2d BLUE_PASS_DEPOT_POSE = new Pose2d(2.25, 6, new Rotation2d(0));
        public static final Pose2d BLUE_PASS_OUTPOST_POSE = new Pose2d(2.25, 2, new Rotation2d(0));
        
        public static final Pose2d BLUE_HUB_CENTER = new Pose2d(3.439, 3.987, new Rotation2d(0));
        public static final Pose2d BLUE_OUTPOST_CENTER = new Pose2d(0.975, 0.630, new Rotation2d(0)); // 180? Intake will either be facing or away from outpost
        public static final Pose2d BLUE_TOWER_CENTER = new Pose2d(1.567, 3.739, new Rotation2d(0));
        public static final Pose2d BLUE_DEPOT_CENTER = new Pose2d(1.212, 5.923, new Rotation2d(0));
        public static final Pose2d BLUE_DEPOT_CORNER = new Pose2d(0.491, 7.074, new Rotation2d(270));
        
        public static final Pose2d BLUE_NEUTRAL_LEFT = new Pose2d(7.775, 6.902, new Rotation2d(90)); // Check if intake is facing fuel
        public static final Pose2d BLUE_NEUTRAL_RIGHT = new Pose2d(7.775, 0.877, new Rotation2d(0)); // Check if intake is facing fuel
        public static final Pose2d BLUE_TRENCH_LEFT = new Pose2d(3.504, 7.559, new Rotation2d(0)); // Check if intake is facing neutral zone
        public static final Pose2d BLUE_TRENCH_RIGHT = new Pose2d(3.504, 0.436, new Rotation2d(180)); // Check if intake is facing neutral zone (or could face the opposite direction)
        
        public static final Pose2d BLUE_HALF_LEFT = new Pose2d(2.159, 5.588, new Rotation2d(45)); // Experimental, we can use these as mid-field shots
        public static final Pose2d BLUE_HALF_RIGHT = new Pose2d(2.159, 2.568, new Rotation2d(135)); // Experimental, we can use these as mid-field shots

        public static final Pose2d BLUE_TRENCH_DEPOT_AUTO_RETURN = new Pose2d(6.118, 7.279, new Rotation2d(270)); // Pose to return to after going to neutral zone for auto
        public static final Pose2d BLUE_TRENCH_OUTPOST_AUTO_RETURN = new Pose2d(6.118, 0.780, new Rotation2d(270)); // Pose to return to after going to neutral zone for auto



        public static final Pose2d RED_TRENCH_OUTPOST_AUTO_RETURN = new Pose2d(10.841, 7.300, new Rotation2d(180));
        public static final Pose2d RED_TRENCH_DEPOT_AUTO_RETURN = new Pose2d(10.841, 0.716, new Rotation2d(90));

        //RED SIDE
        public static final Pose2d RED_HUB_POSE = new Pose2d(12, 4, new Rotation2d(0));
        public static final Pose2d RED_PASS_DEPOT_POSE = new Pose2d(14.25, 2, new Rotation2d(0));
        public static final Pose2d RED_PASS_OUTPOST_POSE = new Pose2d(14.25, 6, new Rotation2d(0));

        public static final Pose2d RED_HUB_CENTER = new Pose2d(13.112, 4.062, new Rotation2d(0));
        public static final Pose2d RED_OUTPOST_CENTER = new Pose2d(15.500, 7.386, new Rotation2d(0)); // 180? Intake will either be facing or away from outpost
        public static final Pose2d RED_TOWER_CENTER = new Pose2d(14.887, 4.352, new Rotation2d(0));
        public static final Pose2d RED_DEPOT_CENTER = new Pose2d(15.296, 2.114, new Rotation2d(0));
        public static final Pose2d RED_DEPOT_CORNER = new Pose2d(15.920, 0.963, new Rotation2d(90));
        
        public static final Pose2d RED_NEUTRAL_LEFT = new Pose2d(8.905, 1.114, new Rotation2d(90)); // Check if intake is facing fuel
        public static final Pose2d RED_NEUTRAL_RIGHT = new Pose2d(8.905, 7.096, new Rotation2d(270)); // Check if intake is facing fuel
        public static final Pose2d RED_NEUTRAL_LEFT_PICKUP = new Pose2d(8.905, 4.740, new Rotation2d(90)); // Check if intake is facing fuel
        public static final Pose2d RED_NEUTRAL_RIGHT_PICKUP = new Pose2d(8.905, 2.695, new Rotation2d(270)); // Check if intake is facing fuel
        public static final Pose2d RED_TRENCH_LEFT = new Pose2d(12.993, 0.619, new Rotation2d(180)); // Check if intake is facing neutral zone
        public static final Pose2d RED_TRENCH_RIGHT = new Pose2d(12.993, 7.408, new Rotation2d(180)); // Check if intake is facing neutral zone (or could face the opposite direction)
        
        public static final Pose2d RED_HALF_LEFT = new Pose2d(14.241, 2.535, new Rotation2d(135)); // Experimental, we can use these as mid-field shots
        public static final Pose2d RED_HALF_RIGHT = new Pose2d(14.241, 5.519, new Rotation2d(45)); // Experimental, we can use these as mid-field shots
    } 

    // wheel radius, max speed, wheel COF, DCMotor drive, drive current limit, # motors
    public static final ModuleConfig moduleConfig = new ModuleConfig(ModuleConstants.kWheelDiameterMeters/2, AutoConstants.kMaxSpeedMetersPerSecond, ModuleConstants.kWheelCOF, ModuleConstants.kDriveMotor, ModuleConstants.kDriveMotorCurrentLimit, ModuleConstants.kNumMotorsPerModule);
    public static final RobotConfig robotConfig = new RobotConfig(RobotConstants.kRobotTotalWeightKG, RobotConstants.kRobotMOI, moduleConfig, DriveConstants.kDriveKinematics.getModules());
    public static final PathConstraints pathConstraints = new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared, AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecond);
}
