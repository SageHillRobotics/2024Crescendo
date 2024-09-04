package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase{
    private static final  double AMP_ENCODER_VALUE = 2.3;
    private static final double INTAKE_ENCODER_VALUE = 1.15;
    private static final double SOURCE_ENCODER_VALUE = 1.16;
    private static final double CLIMB_ENCODER_POSITION = 1.8;
    private static final double FINAL_CLIMB_POSITION = 1;
    private final CANSparkMax leader;
    private final CANSparkMax topLeft;
    private final CANSparkMax bottomLeft;
    private final CANSparkMax bottomRight;
    private final CANSparkMax winch;
    private final RelativeEncoder leaderIntegratedEncoder;
    private final RelativeEncoder throughBoreEncoder;
    private final SparkPIDController pidController;
    private final DigitalInput limitSwitch;
    
    public Elevator(){

        leader = new CANSparkMax(53, MotorType.kBrushless);
        bottomRight = new CANSparkMax(54, MotorType.kBrushless);
        topLeft = new CANSparkMax(52, MotorType.kBrushless);
        bottomLeft = new CANSparkMax(51, MotorType.kBrushless);
        winch = new CANSparkMax(62, MotorType.kBrushless);

        leader.restoreFactoryDefaults();
        bottomRight.restoreFactoryDefaults();
        topLeft.restoreFactoryDefaults();
        bottomLeft.restoreFactoryDefaults();

        leader.setIdleMode(IdleMode.kBrake);
        bottomRight.setIdleMode(IdleMode.kBrake);
        topLeft.setIdleMode(IdleMode.kBrake);
        bottomLeft.setIdleMode(IdleMode.kBrake);

        winch.setIdleMode(IdleMode.kBrake);

        bottomRight.follow(leader);
        topLeft.follow(leader, true);
        bottomLeft.follow(leader, true);

        leaderIntegratedEncoder = leader.getEncoder();
        throughBoreEncoder = leader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        throughBoreEncoder.setInverted(true);

        pidController = leader.getPIDController();
        pidController.setFeedbackDevice(throughBoreEncoder);
        
        pidController.setP(0.17);
        pidController.setI(0);
        pidController.setD(.15);
        pidController.setFF(.07);
        pidController.setOutputRange(-1, 1);

        leader.burnFlash();
        bottomRight.burnFlash();
        topLeft.burnFlash();
        bottomLeft.burnFlash();
        
        limitSwitch = new DigitalInput(9);
    }

    public void intakePosition(){
        pidController.setReference(INTAKE_ENCODER_VALUE, ControlType.kPosition);
    }
    public void ampPosition(){
        pidController.setReference(AMP_ENCODER_VALUE, ControlType.kPosition);
    }
    public void retract(){
        pidController.setReference(0.2, ControlType.kPosition);
    }

    public void climbPosition(){
        pidController.setReference(CLIMB_ENCODER_POSITION, ControlType.kPosition);
    }

    public void sourcePosition(){
        pidController.setReference(SOURCE_ENCODER_VALUE, ControlType.kPosition);
    }
    public boolean atIntakePosition(){
        return Math.abs(throughBoreEncoder.getPosition() - INTAKE_ENCODER_VALUE) < .1;
    }
    public boolean atAmpPosition(){
        return Math.abs(throughBoreEncoder.getPosition() - AMP_ENCODER_VALUE) < .1;
    }
    public double getPosition(){
        return throughBoreEncoder.getPosition();
    }
    public void setSpeed(double speed){
        leader.set(speed);
    }
    public void stop(){
        leader.stopMotor();
        winch.stopMotor();
    }
    public void retractWinch(){
        leader.stopMotor();
        leader.setIdleMode(IdleMode.kCoast);
        bottomRight.setIdleMode(IdleMode.kCoast);
        topLeft.setIdleMode(IdleMode.kCoast);
        bottomLeft.setIdleMode(IdleMode.kCoast);
        winch.set(-.50);
    }
    public void unwindWinch(){
        winch.set(.25);
    }
    public boolean atWinchPosition(){
        return throughBoreEncoder.getPosition() <= FINAL_CLIMB_POSITION;
    }
    public boolean atClimbPosition(){
        return Math.abs(throughBoreEncoder.getPosition() - 1.9436) < .2;
    }
    


    @Override
    public void periodic(){
        SmartDashboard.putNumber("through bore encoder value", throughBoreEncoder.getPosition());
        SmartDashboard.putBoolean("limit switch", limitSwitch.get());
        
        if (!limitSwitch.get()){
            leaderIntegratedEncoder.setPosition(0);
            throughBoreEncoder.setPosition(0);
        }
        
    }
}