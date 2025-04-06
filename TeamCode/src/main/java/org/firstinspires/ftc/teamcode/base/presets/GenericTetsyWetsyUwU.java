package org.firstinspires.ftc.teamcode.base.presets;

import static org.firstinspires.ftc.teamcode.base.Components.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Components;

import org.firstinspires.ftc.teamcode.base.NonLinearActions;

import java.util.ArrayList;
import java.util.Objects;

public class GenericTetsyWetsyUwU extends LinearOpMode {
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
    public <E extends Components.RunConfiguration> void run(){
        E.initialize(hardwareMap,telemetry);
        for (String name:actuators.keySet()){
            if (!(actuators.get(name) instanceof Components.BotMotor)){
                Objects.requireNonNull(actuators.get(name)).switchControl(Objects.requireNonNull(actuators.get(name)).defaultControlKey);
            }
            actuatorNames.add(name);
        }
        NonLinearActions.ConditionalPair[] conditions = new NonLinearActions.ConditionalPair[actuatorNames.size()];
        for (int i=0;i<actuatorNames.size();i++){
            int finalI = i;
            conditions[i]=new NonLinearActions.ConditionalPair(
                    ()->(selectedActuatorIndex==finalI),
                    Objects.requireNonNull(actuators.get(actuatorNames.get(i))).triggeredDynamicAction(()->(gamepad1.left_bumper),()->(gamepad1.right_bumper),1)
            );
        }

        waitForStart();
        NonLinearActions.runLoop(
                this::opModeIsActive,
                new NonLinearActions.RunLoopRoutine<E>(this::updateTelemetry),
                new NonLinearActions.PressTrigger(new NonLinearActions.ConditionalPair(
                        ()->(gamepad1.dpad_left),
                        new NonLinearActions.InstantAction(this::shiftSelectionLeft)
                )),
                new NonLinearActions.PressTrigger(new NonLinearActions.ConditionalPair(
                        ()->(gamepad1.dpad_right),
                        new NonLinearActions.InstantAction(this::shiftSelectionRight)
                )),
                new NonLinearActions.ConditionalAction(
                        conditions
                )
        );
    }
}
