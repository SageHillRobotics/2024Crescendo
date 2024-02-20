package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase{
    private static final  double EXTENDED_ENCODER_VALUE = 4800;
    private final CANSparkMax topLeft;
    private final CANSparkMax bottomLeft;
    private final CANSparkMax topRight;
    private final CANSparkMax bottomRight;
    private final Encoder revThroughBoreEncoder;
    private final PIDController pidController;
    private final DigitalInput limitSwitch;
    
    public Elevator(){
        topLeft = new CANSparkMax(51, MotorType.kBrushless);
        bottomLeft = new CANSparkMax(52, MotorType.kBrushless);
        topRight = new CANSparkMax(53, MotorType.kBrushless);
        bottomRight = new CANSparkMax(54, MotorType.kBrushless);

        topLeft.setSmartCurrentLimit(30);
        bottomLeft.setSmartCurrentLimit(30);
        topRight.setSmartCurrentLimit(30);
        bottomRight.setSmartCurrentLimit(30);

        revThroughBoreEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
        revThroughBoreEncoder.setDistancePerPulse(1./EXTENDED_ENCODER_VALUE);

        bottomRight.follow(topRight);
        topLeft.follow(topRight, true);
        bottomLeft.follow(topRight, true);

        pidController = new PIDController(.000075, 0, 0);

        limitSwitch = new DigitalInput(9);
        
    }

    public void setSpeed(double speed){
        topRight.set(speed);
    }

    public void extendPID(){
        pidController.setP(.5);
        pidController.setI(.2);
        pidController.setD(.4);
        double output = pidController.calculate(revThroughBoreEncoder.getDistance(), 1);
        topRight.set(output);
    }
    public void retractPID(){
        pidController.setP(.1);
        pidController.setI(0);
        pidController.setD(0);
        double output = pidController.calculate(revThroughBoreEncoder.getDistance(), 0);
        topRight.set(output);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("through bore encoder pulses", revThroughBoreEncoder.get());
        SmartDashboard.putNumber("through bore encoder distance", revThroughBoreEncoder.getDistance());
        SmartDashboard.putBoolean("limit seitch", limitSwitch.get());
        if (!limitSwitch.get()){
            revThroughBoreEncoder.reset();
        }
    }
}