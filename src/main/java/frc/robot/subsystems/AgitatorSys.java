package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.ShooterSys;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

public class AgitatorSys extends SubsystemBase {
    
    private final SparkMax agitatorMtr;
    private final RelativeEncoder agitatorEnc;
    private final SparkMax feederMtr;
    private final RelativeEncoder feederEnc;
    private final ShooterSys shooterSys;


    public AgitatorSys (ShooterSys shooterSys) {

        this.shooterSys = shooterSys;

        agitatorMtr = new SparkMax(CANDevices.agitatorMtrId, MotorType.kBrushless);
        agitatorEnc = agitatorMtr.getEncoder();
        feederMtr = new SparkMax(CANDevices.feederMtr, MotorType.kBrushless);
        feederEnc = feederMtr.getEncoder();

        SparkMaxConfig agitatorConfig = new SparkMaxConfig();
        agitatorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        agitatorConfig.encoder
            .positionConversionFactor(25)
            .velocityConversionFactor(25);

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        feederConfig.encoder
            .positionConversionFactor(0.33333333)
            .velocityConversionFactor(0.33333333);

        feederMtr.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    /**
     * Sets the agitator motor to the specified RPM. Positive RPMs should intake balls, while negative RPMs should outtake balls.
     */
    public void setAgitatorRPM(boolean reverse) {
        if(reverse == true) {
            agitatorMtr.set(-0.75);
            feederMtr.set(-0.20);
        }else{
            if(shooterSys.getDistanceCenterHub() > 9){
            agitatorMtr.set(0.75);
            feederMtr.set(0.20);
        } else {
            agitatorMtr.set(0.75);
            feederMtr.set(0.40);
        }
        }
    }

    /** Returns the current RPM of the agitator motor. */
    public void getAgitatorRPM() {
        agitatorEnc.getVelocity();
    }

    /** Stops the agitator motor.*/
    public void stop() {
        agitatorMtr.stopMotor();
        feederMtr.stopMotor();
    }

}
