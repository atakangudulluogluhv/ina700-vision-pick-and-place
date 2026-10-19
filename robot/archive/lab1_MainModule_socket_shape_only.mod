MODULE MainModule
    ! Socket-driven version of reference_lab_MainModule.mod: the shape comes
    ! from the Raspberry Pi server instead of the FlexPendant menu.
    !
    ! Connection, retry and timeout come from ServerCommuncatio.sys, which must
    ! stay untouched (T18). Protocol is strict lockstep, server speaks first:
    !   Pi -> robot : "HEXAGON" | "CIRCLE" | "TRIANGLE" | "STAR"
    !   robot -> Pi : "OK" after the place, "FAIL" for an unknown shape
    ! This matches comm.message_template "{shape_upper}" and
    ! comm.ack_success_token "OK" in config/settings.yaml.

    ! Tool, work objects and taught points copied unchanged from the lab module.
    TASK PERS tooldata tool1:=[TRUE,[[93.4135,-26.9322,198.73],[1,0,0,0]],[1.1,[20,20,25],[1,0,0,0],0,0,0]];
    TASK PERS wobjdata wobj1:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[947.997,-298.397,611.543],[1,-2.34442E-06,-7.35128E-06,8.72059E-06]]];
    TASK PERS wobjdata wobj1_2:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[720.997,157.430,611.543],[1,-2.34442E-06,-7.35128E-06,8.72059E-06]]];
    CONST robtarget p10:=[[899.80,-175.06,466.14],[0.00208589,-0.00550162,-0.999934,0.00987325],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p20:=[[-54.61,118.18,-167.32],[0.00211613,-0.0054715,-0.999934,0.00988885],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p30:=[[-54.61,118.20,-210.48],[0.00213764,-0.00544815,-0.999934,0.00989338],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p40:=[[13.45,64.60,-161.67],[0.00206241,-0.00542847,-0.999937,0.00956261],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p50:=[[13.44,57.98,-205.56],[0.00207231,-0.00540738,-0.999937,0.00956585],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p60:=[[-43.06,11.48,-152.93],[0.00205805,-0.00536122,-0.99994,0.00928236],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p70:=[[-43.06,11.49,-206.01],[0.00206531,-0.00534045,-0.999941,0.0092734],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p80:=[[65.79,5.07,-141.74],[0.00203289,-0.00527945,-0.999943,0.00900682],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p90:=[[65.79,5.07,-203.92],[0.00204031,-0.0052779,-0.999943,0.00900522],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! The lab routine waited for the yellow button before gripping. Keep it
    ! TRUE for supervised runs; with it the whole cycle must still finish
    ! inside comm.ack_timeout_sec on the Pi, or the lockstep falls out of step.
    CONST bool USE_OK_BUTTON := TRUE;
    ! Lets the vacuum build up before lifting, otherwise the plate can drop.
    CONST num VACUUM_SETTLE_SEC := 0.3;
    CONST string ACK_OK := "OK";
    CONST string ACK_FAIL := "FAIL";

    PROC main()
        VAR string message;
        RobotAsClientConnect;
        WHILE TRUE DO
            message := RobotClientReciveMessage();
            IF RunShape(message) THEN
                RobotClienSendMessage ACK_OK;
            ELSE
                RobotClienSendMessage ACK_FAIL;
            ENDIF
        ENDWHILE
    ERROR
        ! The Pi server was restarted: reconnect instead of stopping the program.
        IF ERRNO = ERR_SOCK_CLOSED THEN
            RobotClientCloseAndDisconnect;
            RobotAsClientConnect;
            RETRY;
        ENDIF
    ENDPROC

    FUNC bool RunShape(string shape)
        TEST shape
        CASE "HEXAGON":
            PickAndPlace p60, p70;
        CASE "CIRCLE":
            PickAndPlace p20, p30;
        CASE "TRIANGLE":
            PickAndPlace p80, p90;
        CASE "STAR":
            PickAndPlace p40, p50;
        DEFAULT:
            TPWrite "Unknown shape: " + shape;
            RETURN FALSE;
        ENDTEST
        RETURN TRUE;
    ENDFUNC

    ! The same taught points are used in both work objects because the pick
    ! and place fixtures are identical; only the work object differs.
    PROC PickAndPlace(robtarget approach, robtarget grip)
        MoveJ approach, v1000, fine, tool1\WObj:=wobj1;
        MoveJ grip, v1000, fine, tool1\WObj:=wobj1;
        IF USE_OK_BUTTON WaitOkButton;
        Set doValve2;
        WaitTime VACUUM_SETTLE_SEC;
        MoveJ approach, v1000, fine, tool1\WObj:=wobj1;
        MoveJ approach, v1000, fine, tool1\WObj:=wobj1_2;
        MoveJ grip, v1000, fine, tool1\WObj:=wobj1_2;
        Reset doValve2;
        WaitTime VACUUM_SETTLE_SEC;
        MoveJ approach, v1000, fine, tool1\WObj:=wobj1_2;
    ENDPROC

    PROC WaitOkButton()
        SetDO doYellowLamp, 1;
        WaitDI diOkButton, 0;
        WaitDI diOkButton, 1;
        SetDO doYellowLamp, 0;
        WaitDI diOkButton, 0;
        WaitTime 0.2;
    ENDPROC
ENDMODULE
