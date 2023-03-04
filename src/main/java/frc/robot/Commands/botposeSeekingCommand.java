package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Targets;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LemonLight;

import static frc.robot.Constants.LLmultiplier;
import static frc.robot.Constants.desiredDistance;


public class botposeSeekingCommand extends CommandBase {
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final LemonLight lemonLight = LemonLight.getInstance();
    private Targets target;

    public botposeSeekingCommand(DriveTrain driveTrain, LemonLight lemonLight) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.lemonLight);
    }

    @Override
    public void initialize() {
        double[] differences = new double[18];//Find the difference of each target compared to the robot and then store its index then sort it and get the least value;
        double[] botpose = lemonLight.getBotpose();
        for(int i = 0; i < Robot.targets.length; i++){
            differences[i] = Math.abs(Robot.targets[i].getX()-botpose[0]);
        }
        int index = 0;
        double closest = Double.MAX_VALUE;
        for(int i = 0; i < differences.length; i++){
            if(differences[i] < closest){
                closest = differences[i];
                index = i;
            }
        }
        target = Robot.targets[index];
    }

    @Override
    public void execute() {
        double[] botpose = lemonLight.getBotpose();
        double[] leftRight = lr(botpose);
        double fb = fb(botpose);
        leftRight[0] += fb;
        leftRight[1] += fb;
        if(leftRight[0] > 1){
            leftRight[1] -= leftRight[0]-1;
            leftRight[0] = 1;
        } else if(leftRight[1] > 1){
            leftRight[0] -= leftRight[1]-1;
            leftRight[1] = 1;
        }
        driveTrain.setLeftMotors(Math.sqrt(leftRight[1])*LLmultiplier);
        driveTrain.setRightMotors(Math.sqrt(leftRight[0])*LLmultiplier);

    }

    @Override
    public boolean isFinished() {
        //TODO: Making it so it can know if the robot's line is within a certain amount to the target
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    private double calculateDistance(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow((x2-x1), 2)+Math.pow((y2-y1), 2));
    }

    private double calculateSlope(double x1, double y1, double x2, double y2){
        double changeInY = y2-y1;
        double changeInX = x2-x1;
        return changeInY/changeInX;
    }
    private double[] lr(double[] botpose){
        double[] output = {0, 0};
        if(target.getX() > botpose[0]){
            output[0] = (Math.sqrt(target.getX()-botpose[0]));
        }
        if(target.getX() < botpose[0]){
            output[1] = (-Math.abs(Math.sqrt(target.getX()-botpose[0])));
        }
        return output;
    }

    private double fb(double[] botpose){
        if (botpose[1] > Constants.scoringY){
            return (Math.sqrt(calculateDistance(botpose[0], botpose[1], target.getX(), target.getY())/100));
        }
        return 0;
    }
}
