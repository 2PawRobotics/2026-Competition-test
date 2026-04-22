package frc.robot.commands.drivetrain;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSys;

public class AimToAllianceZoneCmd extends Command {

    private final SwerveSys swerveSys;

    private final ProfiledPIDController aimController;


    public AimToAllianceZoneCmd(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;

        aimController = new ProfiledPIDController(
            AutoConstants.autoAimkP, 0.0, AutoConstants.autoAimkD,
            new Constraints(
                AutoConstants.autoAimTurnSpeedRadPerSec,
                AutoConstants.autoAumTurnAccelRadPerSecSq));

        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void initialize() {
        aimController.reset(swerveSys.getHeading().getRadians(), swerveSys.getChassisSpeeds().omegaRadiansPerSecond);
    }
    
    @Override
    public void execute() {

        Rotation2d targetHeading = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromRadians((0)) : Rotation2d.fromRadians(180*Math.PI/180);

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
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}