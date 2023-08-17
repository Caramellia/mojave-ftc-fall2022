package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PowerPlay AutoOp L", group = "Linear Opmode")
public class PowerPlayAutoOp_L extends PowerPlayAutoOp {

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {
        leftDst *= -1.0f;
        super.runOpMode();
    }

}
