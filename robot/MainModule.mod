MODULE MainModule
    ! INA700 final project: camera-guided pick and fixed place.
    !
    ! The Raspberry Pi finds a plate with the trained YOLO26-OBB model, turns
    ! its pixel position into millimetres in wobj2 (the picking paper) and sends one message
    ! per plate. This module only parses that message and moves; all vision,
    ! calibration and angle logic stays on the Pi.
    !
    ! Connection, retry and timeout come from ServerCommuncatio.sys, which
    ! must stay untouched (T18). The robot is the client (T8). Strict
    ! lockstep, the server speaks first, no line terminator:
    !
    !   Pi -> robot : PICK;<id>;<shape>;<color>;<x_mm>;<y_mm>;<z_mm>;<rz_deg>
    !   robot -> Pi : ACK;<id>;OK     plate placed
    !                 ACK;<id>;FAIL   message rejected, nothing moved
    !
    ! Every message stays under RAPID's 80-character string limit (T16).
    ! x/y/z are in wobj2 (paper); rz_deg is the plate's turn relative to its
    ! fixture cavity, measured counter-clockwise seen from above.

    ! ---------------------------------------------------------------------
    ! Tool and work objects
    ! ---------------------------------------------------------------------
    ! Names match what was actually taught on this robot's pendant
    ! (2026-09-28), which is the opposite pairing from the original plan:
    ! wobj1 = the place fixture (mould), wobj2 = the picking paper. Values
    ! below are copied from the pendant, not re-derived.
    TASK PERS tooldata tool1:=[TRUE,[[93.6592,-36.0826,202.462],[1,0,0,0]],[1.1,[0,20,25],[1,0,0,0],0,0,0]];

    ! Place fixture (the black mould), defined from its own three corners
    ! (docs/ROBOT_SETUP.md, step 2B, point 2 -- done). Every place point
    ! below is stored relative to wobj1, so if the fixture moves or turns,
    ! only wobj1 needs to be redefined.
    ! It was taught with tool0 active, and the place points were taught in
    ! that frame. Redefining it with tool1 drops the frame ~209 mm and shifts
    ! it sideways, which drove the tool into the mould (2026-11-01). If it
    ! must be redefined: tool0, the same three corners, Linear jog only.
    TASK PERS wobjdata wobj1:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[712.991,154.924,599.812],[0.999976,0.00178742,-0.00671454,2.01148E-05]]];

    ! Picking paper, taught with tool1 active on the paper's own corners
    ! (origin, along X, along Y). The camera reports positions relative to the
    ! ArUco marker, so the Pi adds the marker centre's position in this frame
    ! (coordinate.marker_offset_*_mm): jog tool1 onto the marker centre with
    ! wobj2 selected and enter the pendant's X/Y in the HMI CALIBRATION tab.
    ! 2026-11-01: paper turned to landscape and wobj2 taught again; the marker
    ! centre read (182.37, 157.86). The ~1.2 deg tilt matches the mould
    ! frame's tilt, so the table itself is probably not quite level.
    ! Value saved from the controller; re-read it on the pendant at the exam.
    TASK PERS wobjdata wobj2:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[710.749,-190.346,380.321],[0.999945,-0.00100401,-0.0104361,0.000635222]]];

    ! ---------------------------------------------------------------------
    ! Place points in wobj1 (mould), one pair per cavity: approach point
    ! above, grip point in the cavity. Re-taught for real (2026-09-28,
    ! docs/ROBOT_SETUP.md step 2B, point 3) after wobj1 was redefined from
    ! the mould's own corners -- these values came from the robot itself
    ! (robot/robot_code_final.mod), not computed here.
    ! ---------------------------------------------------------------------
    CONST robtarget p20:=[[-64.48,133.05,-136.27],[0.00299319,0.124051,0.992254,0.00579034],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p30:=[[-65.23,132.86,-192.01],[0.00298973,0.124054,0.992254,0.00580273],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p40:=[[5.36,74.94,-153.41],[0.0030434,0.124019,0.992258,0.0057717],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p50:=[[4.79,74.80,-195.13],[0.00304501,0.124019,0.992258,0.0057851],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p60:=[[-56.16,24.25,-127.37],[0.003056,0.124012,0.992259,0.00578223],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p70:=[[-57.09,24.02,-196.65],[0.00306251,0.124012,0.992259,0.00581142],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! Pick orientation template: only its orientation and configuration are
    ! used; the position is overwritten from each message. It must equal the
    ! place points' orientation, otherwise the wrist turns between pick and
    ! place and every plate lands rotated by the difference. 2026-10-07: the
    ! old template (from the first lab's p30) was 13.6 deg off the re-taught
    ! p30/p50/p70, so it is now copied from p30.
    CONST robtarget pPickTemplate:=[[0,0,0],[0.00298973,0.124054,0.992254,0.00580273],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! ---------------------------------------------------------------------
    ! Tunables -- change on the pendant, never in the vision code
    ! ---------------------------------------------------------------------
    ! Height above the grip point where every descent and lift starts.
    CONST num APPROACH_HEIGHT_MM := 60;
    ! Extra height for both transfers (paper <-> mould). A MoveJ swings the
    ! TCP along an arc that depends on the start pose, and the approach
    ! points sit only 42-69 mm above the mould; both transfers are therefore
    ! straight MoveL moves this much higher (lab 2026-11-01).
    CONST num TRANSFER_CLEARANCE_MM := 100;
    ! Lets the vacuum build up before lifting and fall away before leaving.
    CONST num VACUUM_SETTLE_SEC := 0.5;
    ! Safety envelope for a camera position in wobj2 (paper). A message outside
    ! it is refused with FAIL instead of being driven to.
    CONST num PICK_REACH_MM := 400;
    CONST num PICK_Z_MIN_MM := -10;
    CONST num PICK_Z_MAX_MM := 100;
    ! The tool's Z axis points down, so a turn about it runs opposite to a
    ! counter-clockwise turn seen from above. Flip to 1 if the lab test shows
    ! the wrist turning the wrong way (docs/ROBOT_SETUP.md, step 4).
    CONST num TOOL_RZ_SIGN := -1;
    ! Supervised runs: wait for the yellow button before each descent. The
    ! Pi waits comm.ack_timeout_sec for the ACK, which covers this wait.
    CONST bool USE_OK_BUTTON := TRUE;

    ! [v_tcp mm/s, v_ori deg/s, v_leax, v_reax] -- same layout as v500/v100.
    CONST speeddata V_TRANSFER := [500,500,5000,1000];
    CONST speeddata V_VERTICAL := [100,500,5000,1000];

    CONST string FIELD_SEPARATOR := ";";
    CONST num PICK_FIELD_COUNT := 8;

    ! ---------------------------------------------------------------------
    ! Parsed message (filled by ParsePick)
    ! ---------------------------------------------------------------------
    VAR string msgId := "0";
    VAR string msgShape := "";
    VAR num msgX := 0;
    VAR num msgY := 0;
    VAR num msgZ := 0;
    VAR num msgRz := 0;

    PROC main()
        VAR string message;
        ! A turned wrist changes axis 6's configuration; without this every
        ! rotated pick would stop with a configuration error.
        ConfJ \Off;
        ConfL \Off;
        RobotAsClientConnect;
        WHILE TRUE DO
            message := RobotClientReciveMessage();
            HandleMessage message;
        ENDWHILE
    ERROR
        ! The Pi server was restarted: reconnect instead of stopping.
        IF ERRNO = ERR_SOCK_CLOSED THEN
            RobotClientCloseAndDisconnect;
            RobotAsClientConnect;
            RETRY;
        ENDIF
    ENDPROC

    PROC HandleMessage(string message)
        VAR robtarget placeApproach;
        VAR robtarget placeGrip;
        IF NOT ParsePick(message) THEN
            TPWrite "Rejected: " + message;
            Reply "FAIL";
            RETURN;
        ENDIF
        IF NOT IsReachable() OR NOT PlaceFor(msgShape, placeApproach, placeGrip) THEN
            Reply "FAIL";
            RETURN;
        ENDIF
        Pick;
        Place placeApproach, placeGrip;
        Reply "OK";
    ENDPROC

    PROC Reply(string verdict)
        RobotClienSendMessage "ACK;" + msgId + ";" + verdict;
    ENDPROC

    ! ---------------------------------------------------------------------
    ! Parsing
    ! ---------------------------------------------------------------------
    FUNC bool ParsePick(string message)
        VAR string fields{PICK_FIELD_COUNT};
        VAR bool numbersOk;
        IF SplitFields(message, fields) <> PICK_FIELD_COUNT THEN
            RETURN FALSE;
        ENDIF
        IF fields{1} <> "PICK" THEN
            RETURN FALSE;
        ENDIF
        msgId := fields{2};
        msgShape := fields{3};
        ! fields{4} is the colour: reported for the log, not needed to move.
        numbersOk := StrToVal(fields{5}, msgX) AND StrToVal(fields{6}, msgY)
                     AND StrToVal(fields{7}, msgZ) AND StrToVal(fields{8}, msgRz);
        RETURN numbersOk;
    ENDFUNC

    ! Splits text at FIELD_SEPARATOR into fields; returns how many were found.
    ! A message with more fields than the array holds counts as too long.
    FUNC num SplitFields(string text, INOUT string fields{*})
        VAR num start := 1;
        VAR num separatorPos;
        VAR num count := 0;
        WHILE start <= StrLen(text) DO
            IF count = Dim(fields, 1) THEN
                RETURN count + 1;
            ENDIF
            separatorPos := StrFind(text, start, FIELD_SEPARATOR);
            count := count + 1;
            fields{count} := StrPart(text, start, separatorPos - start);
            start := separatorPos + 1;
        ENDWHILE
        RETURN count;
    ENDFUNC

    FUNC bool IsReachable()
        IF Abs(msgX) > PICK_REACH_MM OR Abs(msgY) > PICK_REACH_MM THEN
            TPWrite "Outside pick area: x/y";
            RETURN FALSE;
        ENDIF
        IF msgZ < PICK_Z_MIN_MM OR msgZ > PICK_Z_MAX_MM THEN
            TPWrite "Outside pick area: z";
            RETURN FALSE;
        ENDIF
        RETURN TRUE;
    ENDFUNC

    ! Fixed place cavity per assigned shape (A1: circle, hexagon, star).
    FUNC bool PlaceFor(string shape, INOUT robtarget approach, INOUT robtarget grip)
        IF shape = "circle" THEN
            approach := p20;
            grip := p30;
        ELSEIF shape = "star" THEN
            approach := p40;
            grip := p50;
        ELSEIF shape = "hexagon" THEN
            approach := p60;
            grip := p70;
        ELSE
            TPWrite "No place cavity for: " + shape;
            RETURN FALSE;
        ENDIF
        RETURN TRUE;
    ENDFUNC

    ! ---------------------------------------------------------------------
    ! Motion
    ! ---------------------------------------------------------------------
    PROC Pick()
        VAR robtarget grip;
        VAR robtarget above;
        grip := pPickTemplate;
        grip.trans := [msgX, msgY, msgZ];
        ! Turn the gripper with the plate so that, carried to the untouched
        ! place orientation, the plate lines up with its cavity.
        grip := RelTool(grip, 0, 0, 0 \Rz:=TOOL_RZ_SIGN * msgRz);
        above := Offs(grip, 0, 0, APPROACH_HEIGHT_MM);

        MoveL Offs(above, 0, 0, TRANSFER_CLEARANCE_MM), V_TRANSFER, z10, tool1 \WObj:=wobj2;
        MoveL above, V_VERTICAL, fine, tool1 \WObj:=wobj2;
        IF USE_OK_BUTTON WaitOkButton;
        MoveL grip, V_VERTICAL, fine, tool1 \WObj:=wobj2;
        Set doValve2;
        WaitTime VACUUM_SETTLE_SEC;
        MoveL above, V_VERTICAL, fine, tool1 \WObj:=wobj2;
    ENDPROC

    PROC Place(robtarget approach, robtarget grip)
        MoveL Offs(approach, 0, 0, TRANSFER_CLEARANCE_MM), V_TRANSFER, z10, tool1 \WObj:=wobj1;
        MoveL approach, V_VERTICAL, fine, tool1 \WObj:=wobj1;
        MoveL grip, V_VERTICAL, fine, tool1 \WObj:=wobj1;
        Reset doValve2;
        WaitTime VACUUM_SETTLE_SEC;
        MoveL approach, V_VERTICAL, fine, tool1 \WObj:=wobj1;
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
