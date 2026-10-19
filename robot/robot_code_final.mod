MODULE MainModule
    TASK PERS tooldata tool1:=[TRUE,[[93.6592,-36.0826,202.462],[1,0,0,0]],[1.1,[0,20,25],[1,0,0,0],0,0,0]];
    TASK PERS wobjdata wobj1:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[712.991,154.924,599.812],[0.999976,0.00178742,-0.00671454,2.01148E-05]]];
    TASK PERS wobjdata wobj2:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[833.441,0.83,591.391],[0.999966,-0.00138284,-0.00817519,-2.12747E-05]]];

    CONST robtarget p20:=[[-64.48,133.05,-136.27],[0.00299319,0.124051,0.992254,0.00579034],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p30:=[[-65.23,132.86,-192.01],[0.00298973,0.124054,0.992254,0.00580273],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p40:=[[5.36,74.94,-153.41],[0.0030434,0.124019,0.992258,0.0057717],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p50:=[[4.79,74.80,-195.13],[0.00304501,0.124019,0.992258,0.0057851],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p60:=[[-56.16,24.25,-127.37],[0.003056,0.124012,0.992259,0.00578223],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p70:=[[-57.09,24.02,-196.65],[0.00306251,0.124012,0.992259,0.00581142],[0,0,4,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget pPickTemplate:=[[0,0,0],[0.00213764,-0.00544815,-0.999934,0.00989338],[-1,-1,3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    CONST num APPROACH_HEIGHT_MM := 60;
    CONST num VACUUM_SETTLE_SEC := 0.5;
    CONST num PICK_REACH_MM := 400;
    CONST num PICK_Z_MIN_MM := -215;
    CONST num PICK_Z_MAX_MM := 100;
    CONST num TOOL_RZ_SIGN := -1;
    CONST bool USE_OK_BUTTON := TRUE;

    CONST speeddata V_TRANSFER := [500,500,5000,1000];
    CONST speeddata V_VERTICAL := [100,500,5000,1000];

    CONST string FIELD_SEPARATOR := ";";
    CONST num PICK_FIELD_COUNT := 8;

    VAR string msgId := "0";
    VAR string msgShape := "";
    VAR num msgX := 0;
    VAR num msgY := 0;
    VAR num msgZ := 0;
    VAR num msgRz := 0;

    PROC main()
        VAR string message;
        ConfJ \Off;
        ConfL \Off;
        RobotAsClientConnect;
        WHILE TRUE DO
            message := RobotClientReciveMessage();
            HandleMessage message;
        ENDWHILE
    ERROR
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
        IF NOT IsReachable() THEN
            Reply "FAIL";
            RETURN;
        ENDIF
        IF NOT PlaceFor(msgShape, placeApproach, placeGrip) THEN
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
        numbersOk := StrToVal(fields{5}, msgX) AND StrToVal(fields{6}, msgY)
                     AND StrToVal(fields{7}, msgZ) AND StrToVal(fields{8}, msgRz);
        RETURN numbersOk;
    ENDFUNC

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

    PROC Pick()
        VAR robtarget grip;
        VAR robtarget above;
        grip := pPickTemplate;
        grip.trans := [msgX, msgY, msgZ];
        grip := RelTool(grip, 0, 0, 0 \Rz:=TOOL_RZ_SIGN * msgRz);
        above := Offs(grip, 0, 0, APPROACH_HEIGHT_MM);

        MoveJ above, V_TRANSFER, fine, tool1 \WObj:=wobj2;
        IF USE_OK_BUTTON WaitOkButton;
        MoveL grip, V_VERTICAL, fine, tool1 \WObj:=wobj2;
        Set doValve2;
        WaitTime VACUUM_SETTLE_SEC;
        MoveL above, V_VERTICAL, fine, tool1 \WObj:=wobj2;
    ENDPROC

    PROC Place(robtarget approach, robtarget grip)
        MoveJ approach, V_TRANSFER, fine, tool1 \WObj:=wobj1;
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
