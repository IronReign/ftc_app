package com.ironreignrobotics.resq;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.swerverobotics.library.interfaces.TeleOp;

@TeleOp(name = "6832 Config", group = "IronReign")
public class OpFtcConfig extends OpMode {
//

  FtcConfig ftcConfig=new FtcConfig();

  @Override
  public void init() {
    ftcConfig.init(hardwareMap.appContext, this);
  }

  @Override
  public void init_loop() {
    ftcConfig.init_loop(hardwareMap.appContext, this);
  }

  @Override
  public void loop() {
    telemetry.clearData();
    // can use configured variables here
    telemetry.addData("ColorIsRed", Boolean.toString(ftcConfig.param.colorIsRed));
    telemetry.addData("DelayInSec", Integer.toString(ftcConfig.param.delayInSec));
    telemetry.addData("AutonType", ftcConfig.param.autonType);
  }

}
