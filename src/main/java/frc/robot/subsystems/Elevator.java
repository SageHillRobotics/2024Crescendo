package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase{
    private static final  double extendedEncoderValue = 0;
    private final CANSparkMax motor1;
    private final CANSparkMax motor2;
    private final CANSparkMax motor3;
    private final CANSparkMax motor4;
    private final Encoder revThroughBoreEncoder;
    private final PIDController pidController;
    
    public Elevator(){
        motor1 = new CANSparkMax(51, MotorType.kBrushless);
        motor2 = new CANSparkMax(52, MotorType.kBrushless);
        motor3 = new CANSparkMax(53, MotorType.kBrushless);
        motor4 = new CANSparkMax(54, MotorType.kBrushless);
        revThroughBoreEncoder = new Encoder(0, 1);
        pidController = new PIDController(.0001, 0, 0);
    }

    public void setSpeed(double speed){
        motor1.set(speed);
        motor2.set(speed);
        motor3.set(speed);
        motor4.set(speed); 
    }

    public void extendPID(){
        double output = pidController.calculate(revThroughBoreEncoder.getDistance(), extendedEncoderValue);
        motor1.set(output);
        motor2.set(output);
        motor3.set(output);
        motor4.set(output);
    }
    public void retractPID(){
        double output = pidController.calculate(revThroughBoreEncoder.getDistance(), 0);
        motor1.set(output);
        motor2.set(output);
        motor3.set(output);
        motor4.set(output);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("through bore encoder", revThroughBoreEncoder.get());
    }
}