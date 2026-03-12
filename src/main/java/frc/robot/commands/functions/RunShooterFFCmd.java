package frc.robot.commands.functions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSys;

public class RunShooterFFCmd extends Command {
    private final ShooterSys shooter;
    private final SimpleMotorFeedforward ff;

    // require shooter so command has exclusive control
    public RunShooterFFCmd(ShooterSys shooter) {
        this.shooter = shooter;
        // ff constants: ks, kv, ka. Ensure kv/ka units match rad/s (see note below)
        this.ff = new SimpleMotorFeedforward(0, 0.0175, 0);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // convert RPM -> rad/s
        double targetRadPerSec = shooter.desiredRPM() * 2.0 * Math.PI / 60.0;

        // Feedforward (volts) for target vel
        double ffVolts = ff.calculate(targetRadPerSec);

        double volts = ffVolts;

        // clamp to battery safe range
        double maxV = RobotController.getBatteryVoltage();
        volts = MathUtil.clamp(volts, -maxV, maxV);

        // write voltage via motor controller (voltage control)
        shooter.setControllerVolts(volts);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted; change to finish when on-target if you want
    }
}
