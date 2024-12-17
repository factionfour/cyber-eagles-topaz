package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Topaz Teleop V3", group="Robot")

public class RobotTeleopTank_Iterative_Std extends TeleopBase {
    @Override
    public void init() {
        initializeHardware();
    }

    @Override
    public void loop() {
        driveWheels();
        moveIntake(gamepad2.left_bumper,gamepad2.right_bumper);
        moveArm(gamepad2.left_stick_y);
        moveExtension(gamepad2.right_stick_y);

        // --- STOP & EMERGENCY ACTIONS
        if (gamepad2.back) {
            emergencyReset();
        }

        // --- AUTOMATED MOVEMENT BUTTONS
        if (gamepad2.y) {
            //SPECIMEN HOOK
            if (specimenHookState == HookState.IDLE) {
                specimenHookState = HookState.POSITION_ROBOT; // Start first step
                tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                tmpArmPositionHolder = armMotor.getCurrentPosition();
            }

            // Execute multi-step process based on current state
            switch (specimenHookState) {
                case POSITION_ROBOT:
                    specimenHookState = HookState.PLACE_ARM; // Transition to next step
                case PLACE_ARM:
                    if (moveArmEncoder(tmpArmPositionHolder,HOOK_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder,HOOK_EXTENSION_POSITION)) {
                        specimenHookState = HookState.MOVE_OUT; // Transition to next step
                        tmpActionStartTime = System.currentTimeMillis();
                        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    }
                    break;
                case MOVE_OUT:
                    long elapsedTime = System.currentTimeMillis() - tmpActionStartTime;
                    if (elapsedTime <= 500) {
                        moveIntake(false, true);
                    }
                    else {
                        moveIntake(false,false);
                        specimenHookState = HookState.HOOK_ARM; // Transition to next step
                    }
                case HOOK_ARM:
                    if (moveExtensionEncoder(tmpExtensionPositionHolder,0)) {
                        specimenHookState = HookState.COMPLETE; // Transition to next step
                    }
                    break;
                case COMPLETE:
                    setDefaultPower();
                    break;
            }
        }
        else {
            specimenHookState = HookState.IDLE; // Transition to next step
        }

        if (gamepad2.b) {
            //SAMPLE RELEASE TO FIRST BUCKET
            if (sampleReleaseState == releaseSampleFirstBucketState.IDLE) {
                sampleReleaseState = releaseSampleFirstBucketState.POSITION_ROBOT; // Start first step
                tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                tmpArmPositionHolder = armMotor.getCurrentPosition();
            }

            // Execute multi-step process based on current state
            switch (sampleReleaseState) {
                case POSITION_ROBOT:
                    //TODO: BUILD CODE TO MOVE ROBOT TO SPECIFIC POSITION IN MAP BASED ON TELEMETRY
                    sampleReleaseState = releaseSampleFirstBucketState.MOVE_ARM; // Transition to next step
                case MOVE_ARM:
                    if (moveArmEncoder(tmpExtensionPositionHolder,RELEASE_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpArmPositionHolder,RELEASE_SAMPLE_EXTENSION_POSITION)) {
                        sampleReleaseState = releaseSampleFirstBucketState.COMPLETE; // Transition to next step
                        tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                    }
                    break;
                case COMPLETE:
                    setDefaultPower();
                    break;
            }

        }
        else {
            sampleReleaseState = releaseSampleFirstBucketState.IDLE; // Transition to next step
        }

        if (gamepad2.a) {
            //SAMPLE PICKUP FROM GROUND
            if (samplePickupState == pickupSampleGroundState.IDLE.IDLE) {
                samplePickupState = pickupSampleGroundState.POSITION_ROBOT; // Start first step
                tmpExtensionPositionHolder = extensionArmMotor.getCurrentPosition();
                tmpArmPositionHolder = armMotor.getCurrentPosition();
            }

            // Execute multi-step process based on current state
            switch (samplePickupState) {
                case POSITION_ROBOT:
                    //TODO: Add code to find the object based on object recognition
                    samplePickupState = pickupSampleGroundState.MOVE_ARM; // Transition to next step
                    break;
                case MOVE_ARM:
                    if (moveArmEncoder(tmpExtensionPositionHolder,PICKUP_SAMPLE_ARM_HEIGHT) && moveExtensionEncoder(tmpExtensionPositionHolder,0)) {
                        samplePickupState = pickupSampleGroundState.INTAKE; // Transition to next step
                    }
                    break;
                case INTAKE:
                    moveIntake(false,false);
                    break;
            }

        }
        else {
            if (samplePickupState != pickupSampleGroundState.IDLE) {
                moveIntake(false,false);
                samplePickupState = samplePickupState.IDLE;
            }
        }
        // --- END AUTOMATED MOVEMENT BUTTONS

        addTelemetry();
    }

}
