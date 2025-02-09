// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import java.sql.Driver;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Analog0Subsystem;

import frc.robot.commands.ClimbStickCommand;
import frc.robot.commands.ShoulderAngleCommand;
import frc.robot.commands.ShoulderJoyStickCommand;
//import frc.robot.commands.AlignWithTargetCommand;
import frc.robot.commands.CANdleBaseCommand;
import frc.robot.commands.ElevatorFixedLocationCommand;
import frc.robot.commands.CANdle_Default;

public class RobotContainer 
{

    private boolean isAnalogActivated = false; 

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController UpperController = new CommandXboxController(1);
    private final PhotonCamera camera = new PhotonCamera("Team_1"); 

    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    double ElevatorControl_High = 205.0;     //POVUP   telop   
    double ElevatorControl_Mid  = 100.0;     //POVLeft  telop
    double ElevatorControl_Home =  1.0;     //POVDOWN  teleop   
    
    double ShoulderHigh = 20.00;
    double ShoulderMed  = 100.00;
    double ShoulderHome = 0.0;
    double SouulderClimbPosition = 0.0;

   
    private final   CANdleSubsystem       m_Candle           = new CANdleSubsystem();
    private final   ClimbSubsystem        ClimbSubsystem     = new ClimbSubsystem();
    private final   ShoulderSubsystem     ShoulderSubsystem  = new ShoulderSubsystem();
    private final   PhotonVisionSubsystem photonVisionSubsystem  = new PhotonVisionSubsystem("Team_1", new Transform3d());
    private final   Analog0Subsystem      Analog0Subsystem   = new Analog0Subsystem ();  
    public  final   ElevatorSubsystem     ElevatorSubsystem   = new ElevatorSubsystem();  
    

    private static final Command ClimbStickCommand = null;    
    //private static final Command ElevatorFixedLocationCommand = null;


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    

    public RobotContainer() 
    {
        
      CommandScheduler.getInstance().setDefaultCommand(ClimbSubsystem, ClimbStickCommand);
      

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() 
    {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand
        
        (
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Math.abs(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Math.abs(joystick.getLeftX())* MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Math.abs(joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
         )
        );

        SmartDashboard.putNumber("Vision Target Yaw", photonVisionSubsystem.getAngleToBestTarget());
        SmartDashboard.putNumber("Vision Target Distance", photonVisionSubsystem.getDistanceToBestTarget());
        SmartDashboard.putBoolean("Has Vision Target", photonVisionSubsystem.hasTargets());

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(0.5))
    );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        m_Candle.setDefaultCommand(new CANdleBaseCommand(m_Candle));
        
    //******************************************************************/
    //                                Climber Joystick
    //*****************************************************************/
     UpperController.rightStick()
     .onTrue(new ClimbStickCommand(ClimbSubsystem,
                                      () -> MathUtil.applyDeadband(UpperController.getRightY(), 0.01) ));

   /******************************************************************/
   //                                Elevator fixed position
   //*****************************************************************/ 

   UpperController
   .povUp()
   .onTrue(new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorControl_High));

   UpperController
    .povDown()
    .onTrue(new  ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorControl_Home));

 //*******************************************************************/
 //                            Shoulder Fixed position
 //*******************************************************************/
   
     UpperController
     .b()
    .onTrue(new ShoulderAngleCommand(ShoulderSubsystem, ShoulderHome));  

    UpperController
    .y()
    .onTrue(new ShoulderAngleCommand(ShoulderSubsystem, ShoulderHigh)); 
    
    UpperController
    .x()
    .onTrue(new ShoulderAngleCommand(ShoulderSubsystem, ShoulderMed)); 

 //******************************************************************/
 //                                Shoulder Joystick
 //*****************************************************************/
    UpperController.leftStick()
    .onTrue(new ShoulderJoyStickCommand(ShoulderSubsystem,
                                     () -> MathUtil.applyDeadband(UpperController.getLeftY(), 0.01) ));


    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
