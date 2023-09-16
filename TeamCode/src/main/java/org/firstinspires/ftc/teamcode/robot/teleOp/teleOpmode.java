package org.firstinspires.ftc.teamcode.robot.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.robot.utils.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.AngleOffsetHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

import java.io.FileNotFoundException;

@TeleOp(name = "TeleOpProduction")
public class teleOpmode extends LoopUtil {

    public CustomMecanumDrive drive;
    public InputHandler inputHandler;
    public InputHandler gamepadHandler;

    public static Vector3d joystick = new Vector3d();
    public static Vector3d right_stick = new Vector3d();
    public double angleOffset = 0;
    public boolean fieldCentric;

    @Override
    public void opInit() {
        drive = new CustomMecanumDrive(hardwareMap, 1, 1, 1);
        inputHandler = InputAutoMapper.normal.autoMap(this);
        gamepadHandler = InputAutoMapper.normal.autoMap(this);

        AngleOffsetHandler offsetHandler = new AngleOffsetHandler();
        try {
            angleOffset = offsetHandler.fromXML();
        } catch (FileNotFoundException e) {
            angleOffset = 0;
        }
    }


    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        handleInput();
    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        driveFixedUpdate(deltaTime);

    }

    public void driveFixedUpdate(double deltaTime) {
        joystick.x = gamepad1.left_stick_x * -0.6;
        joystick.y = -gamepad1.left_stick_y * -0.6;

        right_stick.x = -gamepad1.right_stick_x;
        right_stick.y = gamepad1.right_stick_y;

        joystick.z =right_stick.x*0.5;
        drive.update( joystick,true, deltaTime, angleOffset);
    }

    @Override
    public void opStop() {

    }

    public void handleInput(){
        inputHandler.loop();
        gamepadHandler.loop();
    }
}
