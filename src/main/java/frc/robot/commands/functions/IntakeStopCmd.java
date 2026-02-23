package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.IntakeSys;

public class IntakeStopCmd extends Command {
    
    private final IntakeSys intakeSys;

    public IntakeStopCmd(IntakeSys intakeSys) {
        this.intakeSys = intakeSys;
    }

    @Override
    public void execute() {
        intakeSys.stop();
    }


}
