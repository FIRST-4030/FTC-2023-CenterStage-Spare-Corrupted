package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;

@TeleOp
public class MecanumTeleOp extends OpMode {
    NewMecanumDrive drive;
    InputHandler inputHandler;
    Vector3d controller;
    @Override
    public void init() {
        drive = new NewMecanumDrive(hardwareMap);
        inputHandler = InputAutoMapper.normal.autoMap(this);
        controller = new Vector3d();

    }

    @Override
    public void loop() {
        handleInput();
        drive.update(controller);
    }

    public void handleInput() {
        inputHandler.loop();
        controller = new Vector3d(gamepad1.left_stick_x , gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
