package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSys;

public class AgitatorCmd extends Command {

    private final AgitatorSys agitatorSys;

    public AgitatorCmd(AgitatorSys agitatorSys) {
        this.agitatorSys = agitatorSys;
    }

    @Override
    public void execute() {
        agitatorSys.setAgitatorRPM();
        agitatorSys.getAgitatorRPM();
    }

    @Override
    public void end(boolean interrupted) {
        agitatorSys.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
