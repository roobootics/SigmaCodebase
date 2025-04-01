package org.firstinspires.ftc.teamcode.base.custom;

import static org.firstinspires.ftc.teamcode.base.Components.timer;


import org.firstinspires.ftc.teamcode.base.Components.BotServo;
import org.firstinspires.ftc.teamcode.base.Components.CRActuator;
public abstract class TimeBasedLocalizers{
    public static class ServoTimeBasedLocalizer{
        public double ABS_SERVO_SPEED;
        public double prevPosition;
        public double prevTime;
        public ServoTimeBasedLocalizer(double servoSpeed){
            this.ABS_SERVO_SPEED=servoSpeed;
        }
        public double getCurrentPosition(BotServo servo){
            if (!servo.notCommanded){
                double servoSpeed = Math.signum(servo.part.getPosition()-prevPosition)*ABS_SERVO_SPEED;
                double time=timer.time();
                double change = servoSpeed*(time-prevTime);
                if (Math.signum(change)==1){
                    prevPosition=Math.min(servo.part.getPosition(),prevPosition+change);
                }
                else if (Math.signum(change)==-1){
                    prevPosition=Math.max(servo.part.getPosition(),prevPosition+change);
                }
                prevTime=time;
            }
            return prevPosition;
        }
    }
    public static class CRTimeBasedLocalizer<E extends CRActuator<?>>{
        public double ABS_SERVO_SPEED;
        public double prevPosition;
        public double prevTime;
        public CRTimeBasedLocalizer(double servoSpeed){
            this.ABS_SERVO_SPEED=servoSpeed;
        }
        public double getCurrentPosition(E actuator){
            double servoSpeed = actuator.part.getPower()*ABS_SERVO_SPEED;
            double time=timer.time();
            prevPosition=prevPosition+servoSpeed*(time-prevTime);
            prevTime=time;
            return prevPosition;
        }
    }
}
