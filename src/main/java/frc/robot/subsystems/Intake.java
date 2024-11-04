package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase{
    private final static double EXTENDED_ENCODER_VALUE = 5;
    private final CANSparkMax intakeCollectionMotor;
    private final CANSparkMax intakeRetractionMotor;
    private final SparkPIDController retractionPID;

    public double kP, kI, kD, kFF;

    public Intake(){
        intakeCollectionMotor = new CANSparkMax(49, MotorType.kBrushless);

        intakeRetractionMotor = new CANSparkMax(50, MotorType.kBrushless);
        intakeRetractionMotor.setInverted(true);

        retractionPID = intakeRetractionMotor.getPIDController();

        kP = .1;
        kI = 0;
        kD = 5;
        kFF = -12;

        retractionPID.setP(kP);
        retractionPID.setI(kI);
        retractionPID.setD(kD);
        retractionPID.setFF(kFF);
        retractionPID.setOutputRange(-1,1);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("Feed Forward", kFF);
    }
    public void retract(){
        retractionPID.setReference(0, ControlType.kPosition);
    }
    public void extend(){
        retractionPID.setReference(EXTENDED_ENCODER_VALUE, ControlType.kPosition);
    }
    public void spinIntakeMotor(){
        intakeCollectionMotor.set(-.5);
    }
    public void stopIntakeMotor(){
        intakeCollectionMotor.stopMotor();
    }
    public boolean atExtendedPosition(){
        return Math.abs(intakeRetractionMotor.getEncoder().getPosition() - EXTENDED_ENCODER_VALUE) < 2;
    }

    public boolean atRetractedPosition(){
        return Math.abs(intakeRetractionMotor.getEncoder().getPosition()) < 5;
    }
    public void eject(){
        intakeCollectionMotor.set(.4);
    }
    public double getCurrent(){
        return intakeCollectionMotor.getOutputCurrent();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("intake encoder value", intakeRetractionMotor.getEncoder().getPosition()); 

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        if((p != kP)) { retractionPID.setP(p); kP = p; }
        if((i != kI)) { retractionPID.setI(i); kI = i; }
        if((d != kD)) { retractionPID.setD(d); kD = d; }
        if((ff != kFF)) { retractionPID.setFF(ff); kFF = ff; }
    }
}
