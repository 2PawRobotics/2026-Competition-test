package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSys;
import frc.robot.RobotContainer;
import frc.robot.Constants.RollerConstants;

public class IntakeCmd extends Command {

    private final IntakeSys intakeSys;
    private final boolean reverse;
    
     public IntakeCmd(IntakeSys intakeSys, boolean reverse) {
        this.intakeSys = intakeSys;
        this.reverse = reverse;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if(RobotContainer.operatorController.leftStick().getAsBoolean() == true){
            if(reverse == true){
                intakeSys.setRollerRPM(-0.3);
            }else{
                intakeSys.setRollerRPM(0.3);
            }
        }else{
            if(reverse == true){
                intakeSys.setRollerRPM(-0.18);
            }else{
                intakeSys.setRollerRPM(0.18);
        }}
    }

    @Override
    public void end(boolean interrupted) {
        intakeSys.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}