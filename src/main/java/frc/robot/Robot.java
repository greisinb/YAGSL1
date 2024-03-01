// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.encoders.SwerveAbsoluteEncoder;
import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;

//Camera Imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//swerve drive telemetry
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;




/**
 * The VM isLoaded configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
//SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  public SwerveDrive swerveDrive;
  public swervelib.SwerveModule[] swerveModules = new swervelib.SwerveModule[4];
  public swervelib.encoders.SwerveAbsoluteEncoder[] swerveEncoders = new SwerveAbsoluteEncoder[4];

  double maximumSpeed = Units.feetToMeters(4.5);
  double turnVar=0;

  public PS4Controller controller;
  public PS4Controller controller2;

  public Solenoid LifterArmsUp;
  public Solenoid LifterArmsDown;
  //public Solenoid rightLifterArm;

  public CANSparkMax l_firingMotor;
  public CANSparkMax r_firingMotor;
  public SparkPIDController l_firingController;
  public SparkPIDController r_firingController;
  public RelativeEncoder l_firingEncoder;
  public RelativeEncoder r_firingEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public double setPoint;


  public DigitalInput loadedInput;

  public CANSparkMax feederMotor;

  public CANSparkMax angleMotor;
  //public SparkPIDController anglePidController;
  public PIDController anglePidController;
  public Encoder angleEncoder;
  public boolean isLoaded;

  public double sTime;
  public double tTime;

  public int desiredArmPosition;
  public double[] armReferences = new double[]{0, 545, 1271};
                                          //firing, amp, seeking
  public double firingSpeed;

    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tv;
    double x;
    double y;
    double targetAquired;

    Solenoid blueTargetLights;
    Solenoid redTargetLights;

    boolean autoFiring;

  @Override   
  public void robotInit() {
    //CameraServer.startAutomaticCapture();
    controller = new PS4Controller(0);
    controller2 = new PS4Controller(1);
    
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    }catch (Exception e){
      throw new RuntimeException(e);
    }

    swerveModules = swerveDrive.getModules();
    for(int i=0;i<4;i++){
      swerveEncoders[i] = swerveModules[i].getAbsoluteEncoder();
    }

    LifterArmsUp = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    LifterArmsDown = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
   
    blueTargetLights = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    redTargetLights = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

    l_firingMotor = new CANSparkMax(32, MotorType.kBrushless);
    r_firingMotor = new CANSparkMax(31, MotorType.kBrushless);

    l_firingController = l_firingMotor.getPIDController();
    r_firingController = r_firingMotor.getPIDController();

    l_firingEncoder = l_firingMotor.getEncoder();
    r_firingEncoder = r_firingMotor.getEncoder();
    
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000195; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    l_firingController.setP(kP);
    l_firingController.setI(kI);
    l_firingController.setD(kD);
    l_firingController.setIZone(kIz);
    l_firingController.setFF(kFF);
    l_firingController.setOutputRange(kMinOutput, kMaxOutput);

    r_firingController.setP(kP);
    r_firingController.setI(kI);
    r_firingController.setD(kD);
    r_firingController.setIZone(kIz);
    r_firingController.setFF(kFF);
    r_firingController.setOutputRange(kMinOutput, kMaxOutput);

    loadedInput = new DigitalInput(0);

    feederMotor = new CANSparkMax(15, MotorType.kBrushed);
    //feederMotor.setInverted(true);

    angleMotor = new CANSparkMax(16, MotorType.kBrushless);
    angleMotor.setInverted(true);
    angleEncoder = new Encoder(1,2);
    angleEncoder.reset();
    anglePidController = new PIDController(.0009, 0, 0);

    desiredArmPosition = 0; //0 isLoaded firing, 1 isLoaded amp, 2 isLoaded seeking
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("AngleMotor Encoder", angleEncoder.getDistance());
    table = NetworkTableInstance.getDefault().getTable("limelight");
    
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    targetAquired = tv.getDouble(0.0);

    isLoaded = loadedInput.get();

    if(targetAquired==1){
      blueTargetLights.set(true);
      redTargetLights.set(true);
    }else if(DriverStation.getAlliance().get() == Alliance.Blue){
      blueTargetLights.set(true);
      redTargetLights.set(false);
    }else if(DriverStation.getAlliance().get() == Alliance.Red){
      blueTargetLights.set(false);
      redTargetLights.set(true);
    }else{
      blueTargetLights.set(false);
      redTargetLights.set(false);
    }

    //post vision to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimeLightTargetAquired", targetAquired);

    SmartDashboard.putNumber("LeftStickX", controller.getLeftX());
    SmartDashboard.putNumber("LeftStickX", controller.getLeftY());
    SmartDashboard.putNumber("RightStickX", controller.getRightX());
    SmartDashboard.putNumber("RightStickY", controller.getRightY());

    SmartDashboard.putNumber("Arm Position", angleEncoder.getDistance());

    SmartDashboard.putBoolean("cross", controller.getCrossButton());
    SmartDashboard.putBoolean("circle", controller.getCircleButton());
    SmartDashboard.putBoolean("square", controller.getSquareButton());
    SmartDashboard.putBoolean("triangle", controller.getTriangleButton());

    SmartDashboard.putNumber("L2", controller.getL2Axis());
    SmartDashboard.putNumber("R2", controller.getR2Axis());

    SmartDashboard.putNumber("POV",controller.getPOV());

    SmartDashboard.putBoolean("DigitalInput get", loadedInput.get());
  }

  @Override
  public void autonomousInit() {
    sTime = Timer.getFPGATimestamp();
    isLoaded = loadedInput.get();
    setPoint = 4200;
    autoFiring = false;
  }
  @Override
  public void autonomousPeriodic(){
    tTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Time", tTime-sTime);
    if(targetAquired!=1){
      swerveDrive.drive(new Translation2d(0*swerveDrive.getMaximumVelocity(),0.0*swerveDrive.getMaximumVelocity()),-.4*swerveDrive.getMaximumAngularVelocity(),true,false);
    }else if(isLoaded || !isLoaded && ((tTime - sTime)<3.8)){
      swerveDrive.drive(new Translation2d(y * .08 * swerveDrive.getMaximumVelocity(),0 *.2 * swerveDrive.getMaximumVelocity()),-x*.02 * swerveDrive.getMaximumAngularVelocity(),true,false);
      if(Math.abs(x)<.6 && Math.abs(y)<.3){
          autoFiring = true;
          firingSpeed = setPoint;

          if(r_firingEncoder.getVelocity()>(setPoint-50)){
            feederMotor.setVoltage(8.0);
          } 

          if(!loadedInput.get()){
            isLoaded = false;
          }
          }else{
            firingSpeed = 0;
            feederMotor.setVoltage(0);
            autoFiring = false;
          }
    }else{
      firingSpeed = 0;
            feederMotor.setVoltage(0);
            autoFiring = false;

            //Zero self
            //Move out of line (constant distance)
    }

    l_firingController.setReference(-firingSpeed, CANSparkMax.ControlType.kVelocity);
    r_firingController.setReference(firingSpeed, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void teleopInit() {
    isLoaded = loadedInput.get();
    setPoint = 5000;
    autoFiring = false;
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Right Stick X Value", controller.getRightX());

    //Auto Aim Drive
    if(controller.getSquareButton())
    {
         swerveDrive.drive(new Translation2d(y * .08 * swerveDrive.getMaximumVelocity(),0 *.2 * swerveDrive.getMaximumVelocity()),-x*.029* swerveDrive.getMaximumAngularVelocity(),true,false);

         if(Math.abs(x)<.1 && Math.abs(y)<.1){
          autoFiring = true;
          firingSpeed = setPoint;

          if(r_firingEncoder.getVelocity()>(setPoint-50)){
            feederMotor.setVoltage(12.0);
          }

          if(!loadedInput.get()){
            isLoaded = false;
          }
          }else{
            firingSpeed = 0;
            feederMotor.setVoltage(0);
            autoFiring = false;
          }
        }
    //Teleop Drive
    else{
          swerveDrive.drive(new Translation2d(-controller.getLeftY()*.8 * swerveDrive.getMaximumVelocity(),-controller.getLeftX() *.8* swerveDrive.getMaximumVelocity()),-controller.getR2Axis()*.8* swerveDrive.getMaximumAngularVelocity(),true,false);

    }
      if(desiredArmPosition==2){
            feederMotor.setInverted(true);
      }else{
            feederMotor.setInverted(false);
      }
      
      if(controller.getR1Button() && desiredArmPosition == 0){
        firingSpeed = setPoint;
            
        if(r_firingEncoder.getVelocity()>((setPoint)-50)){
              feederMotor.setVoltage(12.0);
        }

        if(!loadedInput.get()){
          isLoaded = false;
        }
      }else if(!controller.getR1Button() && desiredArmPosition == 0 && !autoFiring){
            feederMotor.setVoltage(0.0);
            firingSpeed = 0;
      }
      
    if(isLoaded && desiredArmPosition == 2){
            if(angleEncoder.getDistance() > 545){
              feederMotor.setVoltage(0.0);
              desiredArmPosition = 0;
            }
    }else if (!isLoaded && desiredArmPosition == 2){
            feederMotor.setVoltage(8.0); 
     }else if(controller2.getR1Button() && desiredArmPosition == 1){
            feederMotor.setVoltage(12.0);
    }else if(controller2.getR1Button() && desiredArmPosition !=0){
            feederMotor.setVoltage(8.0);
    }else if(!controller2.getR1Button() && desiredArmPosition !=0){
            feederMotor.setVoltage(0.0);
    }
          //desiredArmPosition = 2;
          
          SmartDashboard.putNumber("POV angle", controller.getPOV());
          if(controller2.getPOV()==0){
            desiredArmPosition = 1;
          }else if(controller2.getPOV()==270){
            desiredArmPosition = 0;
          }else if(controller.getL1Button()){
            desiredArmPosition = 2;
          }
      
           if(controller2.getPOV()==90){
            LifterArmsUp.set(false);
            LifterArmsDown.set(true);
          }else{
            LifterArmsUp.set(true);
            LifterArmsDown.set(false);
          }
          SmartDashboard.putNumber("Desired Arm Position", desiredArmPosition);
          
          angleMotor.set(anglePidController.calculate(angleEncoder.getDistance(),armReferences[desiredArmPosition]));
      
          l_firingController.setReference(-firingSpeed, CANSparkMax.ControlType.kVelocity);
          r_firingController.setReference(firingSpeed, CANSparkMax.ControlType.kVelocity);
      
          //if(r_firingEncoder.getVelocity()>SmartDashboard.getNumber("Right Firing Motor Speed", 0)){
            SmartDashboard.putNumber("Right Firing Motor Speed", r_firingEncoder.getVelocity());
          //}
          SmartDashboard.putNumber("Angle Motor Speed", anglePidController.calculate(angleEncoder.getDistance(),armReferences[desiredArmPosition]));
          
        }
  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
