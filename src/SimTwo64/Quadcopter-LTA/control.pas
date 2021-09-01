// New Type Definition

type
  TPID = record
    ref, u: double;
    e, de, Se: double;
    last_e: double;

    Kp, Ki, Kd, Kf: double;
    maxSu: double;
  end;


// Global Variables
var Vnom, Vheli, Vup: double;
    delta:                          double;
    xComp, yComp:                   double;
    desiredZ, desiredX, desiredY:   double;
    desiredRoll, last_desiredRoll:  double;
    desiredPitch, last_desiredPitch:double;
    desiredYaw, last_desiredYaw:    double;
    v, max_calc_v, v1, v2, v3, v4:  double;
    v1bkp, v2bkp, v3bkp, v4bkp:     double;
    zPID, xPID, yPID:               TPID;
    xKp, xKi, xKd:                  double;
    yKp, yKi, yKd:                  double;
    yawKp, yawKi, yawKd:            double;
    last_roll, roll:                double;
    last_pitch, pitch:              double;
    last_yaw, yaw:                  double;
    rollTolerance:                  double;
    pitchTolerance:                 double;
    yawTolerance:                   double;
    xTolerance:                     double;
    yTolerance:                     double;
    u1, u2, u3, u4:                 double;
    m11, m12, m13, m14:             double;
    m21, m22, m23, m24:             double;
    m31, m32, m33, m34:             double;
    m41, m42, m43, m44:             double;
    yawRotMatrix:                   matrix;
    x_old, y_old:                   double;
    kRoll, kPitch:                  double;
    maxAngle:                       double;
    yawInitial:                     double;
    rotateLeft:                     byte;
    rotateRight:                    byte;
    perc:                           double;
    desiredPosVector:               matrix;

function FloatToStrFmt(v: double; fmt: string): string;
begin
  result := format(fmt, [v]);
end;

function FloatToStrFmt4(v: double): string;
begin
  result := format('%.4g', [v]);
end;

procedure closeGrip();
begin
  SetAxisPosRef(0,5,rad(15) );
  SetAxisPosRef(0,4,rad(-15) );
end;

procedure openGrip();
begin
  SetAxisPosRef(0, 5,0);
  SetAxisPosRef(0, 4,0);
end;


function CalcPID(var PID: TPID; new_ref, y: double): double;
begin
  with PID do begin
    ref := new_ref;
    last_e := e;
    e := ref - y;
    de := e - last_e;
    Se := Se + e;
    if ki <> 0 then begin
      Se := sat(Se, maxSu / Ki);
    end;
    u := Kp * e + Ki * Se + Kd * de + Kf * ref;
    
    result := u;
  end;
end;

procedure tal;
begin

    v1 := m11 * u1 + m12 * u2 + m13 * u3 + m14 * u4;
    v2 := m21 * u1 + m22 * u2 + m23 * u3 + m24 * u4;
    v3 := m31 * u1 + m32 * u2 + m33 * u3 + m34 * u4;
    v4 := m41 * u1 + m42 * u2 + m43 * u3 + m44 * u4;

    v1bkp := v1;
    v2bkp := v2;
    v3bkp := v3;
    v4bkp := v4;

    perc  := (v1 + v2 + v3 + v4)/4;

    if (rotateLeft = 1) Then begin
     v1 := v1bkp - 0.7 * perc;
     v3 := v3bkp - 0.7 * perc;
     v2 := v2bkp + 0.7 * perc;
     v4 := v4bkp + 0.7 * perc;
    end;
    
    if (rotateRight = 1) Then begin
     v1 := v1bkp + 0.7 * perc;
     v3 := v3bkp + 0.7 * perc;
     v2 := v2bkp - 0.7 * perc;
     v4 := v4bkp - 0.7 * perc;
    end;
    
    if ( abs(yaw - yawInitial) <= 3 ) Then begin
      rotateLeft  := 0;
      rotateRight := 0;
    end;

    SetAxisVoltageRef(0, 0, v1);
    SetAxisVoltageRef(0, 1, -v2);
    SetAxisVoltageRef(0, 2, v3);
    SetAxisVoltageRef(0, 3, -v4)

    SetRCValue(7,9,FloatToStrFmt4(v1));
    SetRCValue(8,9,FloatToStrFmt4(v2));
    SetRCValue(9,9,FloatToStrFmt4(v3));
    SetRCValue(10,9,FloatToStrFmt4(v4));
end;


//Position Control
procedure posControl;
var posXYZ:                                         TPoint3D;
    //desiredPosVector:                               matrix;
    vR:                                             matrix;
    rotMatrix:                                      matrix;
begin
  desiredPosVector := mzeros(2,1);
    
  posXYZ := GetRobotCenterOfMass(0);

  SetRCValue(2,9,FloatToStrFmt4(posXYZ.x));
  SetRCValue(3,9,FloatToStrFmt4(posXYZ.y));

  Msetv(desiredPosVector,0,0, (desiredX - posXYZ.x));
  Msetv(desiredPosVector,1,0, (desiredY - posXYZ.y));

  //Get attitude angles through rotation Matrix (gsilva MsC thesis, page 31)
  rotMatrix := GetSolidRotMat(0,0);

  last_pitch := pitch;
  last_roll := roll;
  last_yaw := yaw;
  pitch := -ASin2( Mgetv(rotMatrix, 2,0), 1);
  roll  :=  ASin2( Mgetv(rotMatrix, 2,1), cos(pitch) );
  yaw   :=  ASin2( Mgetv(rotMatrix, 1,0), cos(pitch) );

  Msetv(yawRotMatrix,0,0,cos(yaw));
  Msetv(yawRotMatrix,0,1,sin(yaw));
  Msetv(yawRotMatrix,1,0,-sin(yaw));
  Msetv(yawRotMatrix,1,1,cos(yaw));
  vR := Mmult(yawRotMatrix, desiredPosVector);

  last_desiredRoll := desiredRoll;
  last_desiredPitch := desiredPitch;
  desiredRoll :=  - kRoll *  Mgetv(vR, 1, 0);
  desiredPitch :=  kPitch * Mgetv(vR,0, 0);
  
  
  //desiredRoll  := Sat(- kRoll  * Mgetv(vR, 1, 0), rad(8));
  //desiredPitch := Sat(  kPitch * Mgetv(vR, 0, 0), rad(8));

  desiredRoll := sat(desiredRoll, rad(maxAngle));
  desiredPitch := sat(desiredPitch, rad(maxAngle));

  SetRCValue(3,2,FloatToStrFmt4(deg(desiredRoll)));
  SetRCValue(4,2,FloatToStrFmt4(deg(desiredPitch)));

  SetRCValue(7,2,FloatToStrFmt4(Mgetv(desiredPosVector,0,0)));
  SetRCValue(8,2,FloatToStrFmt4(Mgetv(desiredPosVector,1,0)));
    
  SetRCValue(11,2,FloatToStrFmt4(posXYZ.x));
  SetRCValue(12,2,FloatToStrFmt4(posXYZ.y));
    
  SetRCValue(14,9,FloatToStrFmt4(deg(yaw)));
  SetRCValue(13,9,FloatToStrFmt4(deg(pitch)));
  SetRCValue(12,9,FloatToStrFmt4(deg(roll)));
    
    
  //u2 := calcPID(xPID, desiredroll, roll);
  // state feedback
  u2 := -8 * (roll - last_roll) -1 * roll + 0.1 * desiredRoll + 6 * (desiredRoll - last_desiredRoll);
  //u3 := calcPID(yPID, desiredPitch, pitch);
  //u3 := -4 * (pitch - last_pitch) -1 * pitch + 0.1 * desiredPitch + 6 * (desiredPitch - last_desiredPitch);
  u3 := -8 * (pitch - last_pitch) -1 * pitch + 0.1 * desiredPitch + 6 * (desiredPitch - last_desiredPitch);

  u4 :=  40 * (yaw - last_yaw)  + 4 * (yaw - desiredyaw);

  SetRCValue(18,6,FloatToStrFmt4(xPID.e));
  SetRCValue(19,6,FloatToStrFmt4(xPID.Se));
  SetRCValue(20,6,FloatToStrFmt4(xPID.de));

  SetRCValue(21,6,FloatToStrFmt4(u2));
  
  SetRCValue(6,6,FloatToStrFmt4(desiredZ));

//  x_old := posXYZ.x;
//  y_old := posXYZ.y;

end;




// Height Control
procedure zControl;
var z, calc_v: double;
begin
  z := GetSolidZ(0,0);
  SetRCValue(11,6,FloatToStrFmt4(z));
  SetRCValue(6,6,FloatToStrFmt4(desiredZ));

  calc_v := calcPID(zPID, desiredZ, z);

  SetRCValue(12,6,FloatToStrFmt4(zPID.e));
  SetRCValue(13,6,FloatToStrFmt4(zPID.Se));
  SetRCValue(14,6,FloatToStrFmt4(zPID.de));

  SetRCValue(15,6,FloatToStrFmt4(calc_v));


  //v := (0.44 * Vnom) +  calc_v;
  //v := 1 * (0.59 * Vnom) +  calc_v;
  v := 1 * (0.6 * Vnom) +  calc_v;
  if desiredZ = 0 then begin
    v := 0;
  end;

  u1 := v;

  SetRCValue(16,6,FloatToStrFmt4(v));
end;



// this procedure is called periodicaly (default: 40 ms)
procedure Control;
begin
  {
  //State := 0;
  if KeyPressed(VK_ESCAPE) then begin
    desiredZ :=0;
    SetRCValue(6,6,FloatToStr(desiredZ));
    State := 0;
    Vheli := 0;
  end;


  if KeyPressed(ord('1')) then begin desiredZ :=1; SetRCValue(6,6,FloatToStr(desiredZ)); end;
  if KeyPressed(ord('2')) then begin desiredZ :=2; SetRCValue(6,6,FloatToStr(desiredZ)); end;
  if KeyPressed(ord('3')) then begin desiredZ :=3; SetRCValue(6,6,FloatToStr(desiredZ)); end;
    
  if KeyPressed(VK_UP) then State := 1;
  if KeyPressed(VK_DOWN) then State := 2;
   }
  
  if RCButtonPressed(26, 1) then begin
    SetRobotPos(0, GetRCValue(27, 2), GetRCValue(28, 2), GetRCValue(29, 2), rad(GetRCValue(30, 2)));
  end;

  if RCButtonPressed(21, 1) then begin
    desiredX := GetRCValue(22, 2);
    desiredY := GetRCValue(23, 2);
    desiredZ := GetRCValue(24, 2);
    desiredyaw := rad(GetRCValue(25, 2));
  end;

  if RCButtonPressed(32, 1) then begin
    xPID.Kp := GetRCValue(33, 2);
    xPID.Ki := GetRCValue(34, 2);
    xPID.Kd := GetRCValue(35, 2);
  end;

  if RCButtonPressed(37, 1) then begin
    yPID.Kp := GetRCValue(38, 2);
    yPID.Ki := GetRCValue(39, 2);
    yPID.Kd := GetRCValue(40, 2);
  end;
  
  if RCButtonPressed(42, 1) then begin
    zPID.Kp := GetRCValue(43, 2);
    zPID.Ki := GetRCValue(44, 2);
    zPID.Kd := GetRCValue(45, 2);
    zPID.Kf := GetRCValue(46, 2);
  end;

  if RCButtonPressed(24, 5) then begin
    desiredX := 0;
    desiredY := 0;
    desiredZ := 1;
  end;

  if RCButtonPressed(49, 3) then begin
    maxAngle := GetRCValue(49, 2);
  end;
  
  if RCButtonPressed(33, 5) then begin
        SetAxisPosRef(0,5,rad(15) )
        SetAxisPosRef(0,4,rad(-15) )
  end;
  
  if RCButtonPressed(34, 5) then begin
        SetAxisPosRef(0, 5,0)
        SetAxisPosRef(0, 4,0)
  end;
  
  if RCButtonPressed(27, 5) then begin
    yawInitial := yaw;
    SetRCValue(15,9,FloatToStr(deg(yawInitial)));
    rotateLeft := 1;
  end;
  
  if RCButtonPressed(29, 5) then begin
    yawInitial := yaw;
    SetRCValue(15,9,FloatToStrFmt4(deg(yawInitial)));
    rotateRight := 1;
  end;
  
  
  if RCButtonPressed(28, 5) then begin
    rotateLeft  := 0;
    rotateRight := 0;
  end;

  SetRCValue(27,6,FloatToStrFmt4(rotateLeft));
  SetRCValue(29,6,FloatToStrFmt4(rotateRight));

  SetRCValue(27,9,FloatToStrFmt4(xPID.Kp));
  SetRCValue(28,9,FloatToStrFmt4(xPID.Ki));
  SetRCValue(29,9,FloatToStrFmt4(xPID.Kd));
  
  SetRCValue(31,9,FloatToStrFmt4(yPID.Kp));
  SetRCValue(32,9,FloatToStrFmt4(yPID.Ki));
  SetRCValue(33,9,FloatToStrFmt4(yPID.Kd));

  zControl();
  
  posControl();

  tal();
end;


// this procedure is called once when the script is started
procedure Initialize;
begin
  openGrip();

  yawRotMatrix    := mzeros(2,2);
  desiredPosVector:= mzeros(1,2);
  desiredRoll     := 0;
  desiredPitch    := 0;
  desiredYaw      := 0;
  u1              := 0;

  Vnom            := 5.6;
  delta           := 0.99;
  
  {zPID.Kp         := 5;
  zPID.Ki         := 0.01;
  zPID.Kd         := 50;
  zPID.Kf         := 0.9;
  zPID.maxSu      := 0.1;}
  
  
  zPID.Kp         := GetRCValue(43, 2);
  zPID.Ki         := GetRCValue(44, 2);
  zPID.Kd         := GetRCValue(45, 2);
  zPID.Kf         := GetRCValue(46, 2);
  zPID.maxSu      := 0.1;

  xPID.Kp         := 0.2;
  xPID.Ki         := 0.00;
  xPID.Kd         := 10;
  xPID.Kf         := 0;
  xPID.maxSu      := 0.1;

  yPID.Kp         := 0.2;
  yPID.Ki         := 0.00;
  yPID.Kd         := 10;
  yPID.Kf         := 0;
  yPID.maxSu      := 0.1;

  kRoll           := 2.0;
  kPitch          := 2.0;

  max_calc_v      := 6;
  desiredZ        := 1;
  xComp           := 0.5;
  yComp           := 0.5;


  yawKp            := 0;
  yawKi            := 0;
  yawKd            := 0;
  
  m11 :=  1;
  m21 :=  1;
  m31 :=  1;
  m41 :=  1;
  
  m12 :=  0;
  m22 :=  1;
  m32 :=  0;
  m42 :=  -1;
  
  m13 :=  -1;
  m23 :=  0;
  m33 :=  1;
  m43 :=  0;


  m14 :=  1;
  m24 :=  -1;
  m34 :=  1;
  m44 :=  -1;

  maxAngle := GetRCValue(49, 2);
  
  // Update Values Sheet
  SetRCValue(3,6,FloatToStrFmt4(zPID.Kp));
  SetRCValue(4,6,FloatToStrFmt4(zPID.Ki));
  SetRCValue(5,6,FloatToStrFmt4(zPID.Kd));
  SetRCValue(6,6,FloatToStrFmt4(desiredZ));
  SetRCValue(7,6,FloatToStrFmt4(Vnom));
end;












