package org.firstinspires.ftc.teamcode.base.presets;

import static org.firstinspires.ftc.teamcode.base.Components.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Components;

import org.firstinspires.ftc.teamcode.base.NonLinearActions;

import java.util.ArrayList;
import java.util.Objects;
@TeleOp
public class GenericTetsyWetsyUwU extends LinearOpMode { //Used to find the specific positions that we will end up setting actuators to. Excuse the crude, yet traditional, naming convention.
    public int selectedActuatorIndex = 0;
    public ArrayList<String> actuatorNames;
    @Override
    public void runOpMode(){
        //run<RunConfiguration>()
    }
    public void updateTelemetry(){
        telemetry.addLine(actuatorNames.get(selectedActuatorIndex));
        telemetry.addData("position", Objects.requireNonNull(actuators.get(actuatorNames.get(selectedActuatorIndex))).getCurrentPosition());
        telemetry.update();
    }
    public void shiftSelectionRight(){
        if (selectedActuatorIndex<actuatorNames.size()-1){
            selectedActuatorIndex+=1;
        }
    }
    public void shiftSelectionLeft(){
        if (selectedActuatorIndex>0){
            selectedActuatorIndex-=1;
        }
    }
    public <E extends Components.PartsConfig> void run(){
        E.initialize(hardwareMap,telemetry);
        for (String name:actuators.keySet()){
            if (!(actuators.get(name) instanceof Components.BotMotor)){
                Objects.requireNonNull(actuators.get(name)).switchControl(Objects.requireNonNull(actuators.get(name)).defaultControlKey);
            }
            else{
                Components.BotMotor motor = Objects.requireNonNull((Components.BotMotor) actuators.get(name));
                motor.resetEncoders();
                motor.setZeroPowerFloat();
            }
            actuatorNames.add(name);
        }
        NonLinearActions.IfThen[] conditions = new NonLinearActions.IfThen[actuatorNames.size()];
        for (int i=0;i<actuatorNames.size();i++){
            int finalI = i;
            conditions[i]=new NonLinearActions.IfThen(
                    ()->(selectedActuatorIndex==finalI),
                    Objects.requireNonNull(actuators.get(actuatorNames.get(i))).triggeredDynamicAction(()->(gamepad1.left_bumper),()->(gamepad1.right_bumper),1)
            );
        }

        waitForStart();
        NonLinearActions.runLoop(
                this::opModeIsActive,
                new NonLinearActions.RunLoopRoutine<E>(this::updateTelemetry),
                new NonLinearActions.PressTrigger(new NonLinearActions.IfThen(
                        ()->(gamepad1.dpad_left),
                        new NonLinearActions.InstantAction(this::shiftSelectionLeft)
                )),
                new NonLinearActions.PressTrigger(new NonLinearActions.IfThen(
                        ()->(gamepad1.dpad_right),
                        new NonLinearActions.InstantAction(this::shiftSelectionRight)
                )),
                new NonLinearActions.ConditionalAction(
                        conditions
                )
        );
    }
}
