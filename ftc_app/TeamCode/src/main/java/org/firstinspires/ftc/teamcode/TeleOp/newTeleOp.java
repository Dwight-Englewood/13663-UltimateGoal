package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="New Tele-Op",group="TeleOp")
public class newTeleOp extends OpMode {
    DeuxBoot robot = new DeuxBoot();
    double speed;
    boolean buttonAheld = false;
    boolean wobble2Closed = true;
    boolean buttonBheld = false;
    boolean outakeClosed = true;
    boolean buttonYheld = false;
    boolean wobble1Closed = false;
    //boolean buttonXheld = false;
    //boolean spinnerClosed = false;


    @Override
    public void init() {
        robot.initNew(hardwareMap);

        

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robot.runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the r hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        robot.notKevinDrive(gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.left_trigger,
                gamepad1.right_trigger,
                speed);

        robot.intake.setPower(gamepad2.right_stick_y);
        robot.intake1.setPower(gamepad2.right_trigger*0.50);
        robot.intake2.setPower(gamepad2.right_trigger*0.50);
        robot.intake1.setPower(gamepad2.left_trigger*0.40);
        robot.intake2.setPower(gamepad2.left_trigger*0.40);

        if (gamepad1.b) {
            speed = 0.25;
        } else if (gamepad1.a) {
            speed = 1;
        } else if (gamepad1.y) {
            speed = -1;
        }
        telemetry.addData("Speed", speed);

        //if (gamepad2.dpad_right) {
          //  robot.extend.setPosition(0);
        //} else if (gamepad2.dpad_left){
          //  robot.extend.setPosition(1);
        //}

        if (gamepad2.y && !buttonYheld) {
            buttonYheld = true;
            if (wobble1Closed) {
                wobble1Closed = false;
                robot.wobble1.setPosition(1);
            } else {
                wobble1Closed = true;
                robot.wobble1.setPosition(0);
            }
        }

        if (!gamepad2.y) {
            buttonYheld = false;
        }


        if (gamepad2.a && !buttonAheld) {
            buttonAheld = true;
            if (wobble2ClosedClosed) {
                wobble2Closed = false;
                robot.wobble2.setPosition(1);
            } else {
                wobble2Closed = true;
                robot.wobble2.setPosition(0);
            }
        }

        if (!gamepad2.a) {
            buttonAheld = false;
        }

        if (gamepad2.b && !buttonBheld) {
            buttonBheld = true;
            if (outakeClosed) {
                outakeClosed = false;
                robot.outake.setPosition(1);

            } else {
                outakeClosed = true;
            }
        }

        if (!gamepad2.b) {
            buttonBheld = false;
        }

/*        if (gamepad2.x && !buttonXheld) {
            buttonXheld = true;
            if (spinnerClosed) {
                spinnerClosed = false;
                robot.spinner.setPosition(0);
            } else {
                spinnerClosed = true;
                robot.spinner.setPosition(0.35);
            }
        }

        if (!gamepad2.x) {
            buttonXheld = false;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop () {
    }
}

