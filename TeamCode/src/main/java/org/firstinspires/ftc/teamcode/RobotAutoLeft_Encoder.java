//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//@Disabled
//@Autonomous(name="Topaz: Auto Drive Position Left (Encoder)", group="Robot")
//public class RobotAutoLeft_Encoder extends AutoBase {
//    @Override
//    public void runOpMode() {
//        initializeHardware();
//        setInitialPosition();
//
//        //hook the first specimen
//        strafeRight(737,200);
//        driveForward(880,200);
//        moveArmEncoder(HOOK_ARM_HEIGHT,3500,400);
//        moveExtensionEncoder(HOOK_EXTENSION_POSITION,2500,200);
//        moveArmEncoder(HOOK_RELEASE_ARM_HEIGHT,2000,200);
//        moveExtensionEncoder(0,2500,200);
//        moveArmEncoder(0,3500,0);
//        rotateToHeading(initRobotHeading,200);//straighten out
//
//        //park the robot
//        strafeLeft(737,200);
//        driveBackward(880,200);
//
//        closeRobot();
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);
//    }
//}
