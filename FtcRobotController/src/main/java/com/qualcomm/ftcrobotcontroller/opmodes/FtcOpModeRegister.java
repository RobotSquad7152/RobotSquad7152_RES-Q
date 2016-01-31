/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

/**
 * Register Op Modes
 */
public class FtcOpModeRegister implements OpModeRegister {

  /**
   * The Op Mode Manager will call this method when it wants a list of all
   * available op modes. Add your op mode to the list to enable it.
   *
   * @param manager op mode manager
   */
  public void register(OpModeManager manager) {

    /*
     * register your op modes here.
     * The first parameter is the name of the op mode
     * The second parameter is the op mode class property
     *
     * If two or more op modes are registered with the same name, the app will display an error.
     */

    //Test Program
    //manager.register("RStest2", RStest2.class);
    manager.register("Pre_Match_Test", Pre_Match_Test.class);

    manager.register("NullOp", NullOp.class);

    //Tele-op Program
    manager.register("RSTeleOp", RSTeleOp.class);
    manager.register("ServoTest", ServoTest.class);
    manager.register("LinearCameraOp", LinearCameraOp.class);
    //Autonomous Programs
      //Blue Alliance Programs No Delays
      manager.register("B_1_OMR_0", B_1_OMR_0.class);
      manager.register("B_1_OMB_0", B_1_OMB_0.class);
      manager.register("B_3_OMR_0", B_3_OMR_0.class);
      manager.register("B_3_OMB_0", B_3_OMB_0.class);
      //Red Alliance Programs No Delays
      manager.register("R_1_OMB_0", R_1_OMB_0.class);
      manager.register("R_1_OMR_0", R_1_OMR_0.class);
      manager.register("R_3_OMB_0", R_3_OMB_0.class);
      manager.register("R_3_OMR_0", R_3_OMR_0.class);
      //Blue Alliance Programs with 10 second delays
      manager.register("B_1_OMR_10", B_1_OMR_10.class);
      manager.register("B_1_OMB_10", B_1_OMB_10.class);
      manager.register("B_3_OMR_10", B_3_OMR_10.class);
      manager.register("B_3_OMB_10", B_3_OMB_10.class);
      //Red Alliance Programs with 10 second delays
      manager.register("R_1_OMB_10", R_1_OMB_10.class);
      manager.register("R_1_OMR_10", R_1_OMR_10.class);
      manager.register("R_3_OMB_10", R_3_OMB_10.class);
      manager.register("R_3_OMR_10", R_3_OMR_10.class);

    /*
     * Uncomment any of the following lines if you want to register an op mode.
     */


    
    
  }
}
