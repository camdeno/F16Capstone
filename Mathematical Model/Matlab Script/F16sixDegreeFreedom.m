% Cam Osborn
% An Adaptation of Stevens Fortran subroutine

% Constants
 % Data
   S     = 300;
   B     = 30;
   CBAR  = 11.32;
   XCGR  = 0.35;
   HX    = 160;
   RTOD  = 57.29578;
 % Parameters
   AXX  = 9496.0;
   AYY  = 55814.0;
   AZZ  = 63100.0;
   AXZ  = 982.0;
   AXZS = AXZ^2;
   XPQ  = AXZ*(AXX-AYY+AZZ);
   GAM  = AXX*AZZ-AXZ^2;
   XQR  = AZZ*(AZZ-AYY)+AXZS;
   ZPQ  =(AXX-AYY)*AXX+AXZS;
   YPR  = AZZ - AXX;
   weight = 25000.0;
   gd   = 32.17;
   MASS = weight/gd;
   
% Control Vector
%    u(1) = Thtl;
%    u(2) = Elev;
%    u(3) = XCG;
%    u(4) = Land;
VT = .5
alt = .6
Amach = .5
Qbar = .5
% State Vector
    x(1)  = VT;
%    x(2)  = Alpha*RTOD;
%    x(3)  = Beta*RTOD;
%    x(4)  = Phi;
%    x(5)  = Theta;
%    x(6)  = Psi;
%    x(7)  = P;
%    x(8)  = Q;
%    x(9)  = R;
%    
%    x(12) = alt;
%    x(13) = POW;

% Air data computer and engine model I need to modify this 
    ADC(VT,alt,Amach,Qbar);
    Cpow = Tgear(Thtl);
    xd(13) = Pdot(POW,Cpow);
    T = thrust(POW,alt,Amach);

% Look up tables and component buildup
% will need to remove complex look up tables CO
    CXT     = CX(Alpha,EL);
    CYT     = CY(Beta,Ail,Rdr);
    CZT     = CZ(Alpha,Beta,EL);
    DAil    = Ail/20;
    DRdr    = Rdr/30;
    CLT     = CL(Alpha,Beta) + DLDA(Alpha,Beta)*DAil + DLDR(Alpha,Beta)*DLDR;
    CMT     = CM(Alpha,Beta);
    CNT     = CN(Alpha,Beta) + DNDA(Alpha,Beta)*DAil + DNDR(Alpha,Beta)*DRDR;
    
% Add Damping Derivatives
    TVT = 0.5/VT; 
    B2V = B*TVT;
    CQ  = CBar*Q*TVT;
    Damp(Alpha,D);
    CXT = CXT + CQ*D(1);
    CYT = CYT + B2V * (D(2)*R + D(3)*P);
    CZT = CZT + CQ*D(4);
    CLT = CLT + B2V*(D(5)*R + D(6)*P);
    CMT = CMT + CQ*D(7) + CZT*(XCGR-XCG);
    CNT = CNT + B2V*(D(8)*R + D(9)*P) - CYT*(XCGR-XCG)*(CBAR/B);
    
    
    

% 6-DoF Equations, Table 2.5-1 Stevens and Lewis

% Force Equations
    UDot  =  R*V - Q*W - GD*STH + (QS * CXT + T)/m;
    VDot  =  P*W - R*U + GCTH * SPH + AY;
    WDot  =  Q*U - P*V + GCTH * CPH + AZ;
    DUM   = (U*U + W*W);
    xd(1) = (U*UDot + V*VDot + W*WDot)/VT;
    xd(2) = (U*WDot - W*UDot) / DUM;
    xd(3) = (VT*VDot- V*XD(1)) * CBTA / DUM;


    % Udot = (R*V) - (Q*W) - ((g_d)*sin(theta)) + (X_A + X_T)./m;
    % Vdot = (R*U) - (P*W) - ((g_d)*sin(phi)*cos(theta)) + (Y_A + Y_T)./m;
    % Wdot = (Q*U) - (P*V) - ((g_d)*cos(phi)*cos(theta)) + (Z_A + Z_T)./m;
   
% Kinematic Equations
    xd(4) = P + (STH/CTH)*(QSPH + R*CPH);
    xd(5) = Q*CPH - R*SPH;
    xd(6) = (QSPH + R*CPH)/CTH;

    % Phidot = P + tan(theta)*(Q*sin(phi)+R*cos(psi));
    % Thetadot = Q*cos(psi)-R*sin(phi);
    % Psidot = (Q*sin(phi)+R*cos(phi))/cos(theta);
    
 % Moment Equations
    Roll    = QSB*CLT;
    Pitch   = QS*CBAR*CMT;
    Yaw     = QSB*CNT;
    PQ      = p*Q;
    qr      = Q*R;
    QHX     = Q*HX;
    xd(7)   = (XPQ*PQ - XQR*QR + AZZ*Roll + AXZ*(Yaw + QHX))/GAM;   % Pdot
    xd(8)   = (YPR*P*R - AXZ*(P*2 - R*2) + Pitch - R*HX)/AYY;       % Qdot
    xd(9)   = (ZPQ*PQ - XPQ*QR + AXZ*Roll + AXX*(Yaw + QHX))/GAM;   % Rdot
 
 
 
   % Gamma*Pdot = J_xz.*[J_x - J_y + J_z].*P.*Q - [J_z.*(J_z - J_y) + J_xz.^2].*Q.*R + J_z.*l + J_xz.*n;
   % J_y*Qdot = (J_z - J_x)*P*R - J_xz*(P^2 - R^2) + m;
   % Gamma*Rdot = [(J_x - J_y)*J_x + J_xy^2]*P*Q - J_xz*[J_x - J_y + J_z]*Q*R + J_xz*l + J_x*n;
   % Gamma = J_x*J_z - J_xz^2;
    
 % Navigation Equations
    T1  = SPH*CPSI; 
    T2  = CPH*STH; 
    T3  = SPH*SPSI;
    S1  = CTH*CPSI;
    S2  = CTH*SPSI; 
    S3  = T1*STH - CPH*SPSI;
    S4  = T3*STH + CPH*CPSI; 
    S5  = SPH*CTH; 
    S6  = T2*CPSI + T3;
    S7  = T2*SPSI - T1;
    S8  = CPH*CTH;
 
    xd(10) = U*S1  + V*S3 + W*S6; % North speed
    xd(11) = U*S2  + V*S4 + W*S7; % East speed
    xd(12) = U*STH - V*S5 - W*S8; % Vertical speed
 
    %     P_NDot =  U*cos(theta)*cos(psi) + V*(-cos(phi)*sin(psi)+sin(psi)*sin(theta)*cos(psi))
    %                     + W(sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi));
    %     P_EDot = U*cos(theta)*sin(psi) + V*(cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))
    %                     + W(-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi));
    %     hDot = U*sin(theta) - V*(sin(phi)*cos(theta)) - W*(cos(phi)*cos(theta));

    
 % Outputs
    out1 = An
    out2 = -AZ/GD
    Alat = Ay/GD
    
 
 
 
 