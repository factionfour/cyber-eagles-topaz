package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Button Test", group="Robot")

public class buttontest extends OpMode {
    public TouchSensor touchsensor;
   // public HardwareMap hardwareMap = null;
    @Override
    public void init() {

        touchsensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
    }

    @Override
    public void loop() {

        long timestamp = (long) (getRuntime() * 1000);
        telemetry.addData("Timestamp", timestamp);

        telemetry.addData("Button pressed", touchsensor.isPressed());
        telemetry.update();
    }
}