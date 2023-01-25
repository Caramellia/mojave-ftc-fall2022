package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Fast AutoOp Blue, Left", group = "Linear Opmode")
public class PowerPlaySuperAutoOp_BL extends PowerPlaySuperAutoOp {

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {
        this.leftDst *= -1;
        super.runOpMode();
    }

}
