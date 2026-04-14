package frc.robot.commands.drivetrain;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSys;

public class AutoAimHubCmd extends Command {

    private final SwerveSys swerveSys;

    private Translation2d targetTranslation;

    private final ProfiledPIDController aimController;

    private final Timer timer;
    private final double duration = 0.5; // seconds


    public AutoAimHubCmd(SwerveSys swerveSys, double duration) {
        this.swerveSys = swerveSys;

        timer = new Timer();

        aimController = new ProfiledPIDController(
            AutoConstants.autoAimkP, 0.0, AutoConstants.autoAimkD,
            new Constraints(
                AutoConstants.autoAimTurnSpeedRadPerSec,
                AutoConstants.autoAumTurnAccelRadPerSecSq));

        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void initialize() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            targetTranslation = FieldConstants.redAllianceHubPose;
        }
        else {
            targetTranslation = FieldConstants.blueAllianceHubPose;
        }
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
		Translation2d extrapolation = new Translation2d(
            swerveSys.getFieldRelativeVelocity().getX(),
            swerveSys.getFieldRelativeVelocity().getY());
    
        Translation2d extrapolatedTranslation = swerveSys.getPose().getTranslation().plus(extrapolation);
        Translation2d extrapolatedTargetOffset = targetTranslation.minus(extrapolatedTranslation);

        final Rotation2d targetHeading = Rotation2d.fromRadians(MathUtil.angleModulus(extrapolatedTargetOffset.getAngle().getRadians()));
        
        SmartDashboard.putNumber("target heading deg", targetHeading.getDegrees());

        PPHolonomicDriveController.overrideRotationFeedback(() -> targetHeading.getRadians());

        if(Math.abs(swerveSys.getHeading().getDegrees() - targetHeading.getDegrees()) > AutoConstants.autoAimToleranceDeg) {
            double aimRadPerSec = aimController.calculate(swerveSys.getHeading().getRadians(), targetHeading.getRadians());
            swerveSys.setOmegaOverrideRadPerSec(Optional.of(aimRadPerSec));
        }
        else {
            swerveSys.setOmegaOverrideRadPerSec(Optional.of(0.0));
        }
        
	}

    @Override
    public void end(boolean isInterrupted) {
        swerveSys.setOmegaOverrideRadPerSec(Optional.empty());
        // Provide a DoubleSupplier; use NaN to indicate "no override" (PPHolonomicDriveController
        // implementations commonly check for NaN to disable an override).
        PPHolonomicDriveController.overrideRotationFeedback(() -> Double.NaN);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

}