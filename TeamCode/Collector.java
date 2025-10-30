package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="IntakeHold_Single", group="Demo")
public class IntakeHold_Single extends LinearOpMode {
    private DcMotor intake = null;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake"); // name from Robot Configuration
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized - press A to run intake");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                intake.setPower(0.6); // change 0.6 to stronger/weaker as needed
            } else {
                intake.setPower(0.0);
            }
            idle(); // lets the system handle background tasks
        }

        intake.setPower(0.0); // safety: stop motor on exit
    }
}
