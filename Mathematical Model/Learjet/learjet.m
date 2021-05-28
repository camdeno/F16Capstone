%% Calspan Learjet Model (symbolic + numeric)
% 05/11/2021
% Ethan Lew (elew@galois.com)
%
% Taken from:
% Berger, T., Tischler, M., Hagerott, 
% S. G., Cotting, M. C., Gray, W. R., Gresham, J., ... & Howland, J. (2017). 
% Development and Validation of a Flight-Identified Full-Envelope Business Jet 
% Simulation Model Using a Stitching Architecture. In AIAA Modeling and 
% Simulation Technologies Conference (p. 1550).Chicago	


%% Symbols Definition
syms Xu Xw Xq                   % longitudinal force derivatives
syms Zu Zw Zq                   % vertical force derivatives
syms Mu Mw Mq                   % pitch derivatives
syms W0 U0                      % x-z body trim 
syms Theta0                     % pitch trim
syms g                          % gravitational acceleration
syms u w                        % x-z body velocity
syms theta q                    % pitch attitude and pitch rate
syms Xde Xdt Zde Zdt Mde Mdt    % X-Z force derivatives, M pitch derivative
syms Yv Yp Yr                   % lateral force derivatives
syms Lv Lp Lr                   % roll moment derivative
syms Nv Np Nr                   % yaw moment derivative
syms Yda Ydr Lda Ldr Nda Ndr    % Y force derivative, roll and pitch derivatives
syms Vtot                       % total airspeed


%% Linear Model
% longitudinal model
Alon = [[Xu, Xw, Xq-W0, -g*cos(Theta0)];...
        [Zu, Zw, Zq+U0, -g*sin(Theta0)];... 
        [Mu, Mw, Mq, 0]; 
        [0, 0, 1, 0]];
        
Blon = [[Xde, Xdt];... 
        [Zde, Zdt];... 
        [Mde, Mdt];... 
        [0, 0]];
        
% lateral model
Alat = [[Yv, Yp+W0, Yr-U0, g*cos(Theta0)];...
         [Lv, Lp, Lr, 0];...
         [Nv, Np, Nr, 0];...
         [0, 1, tan(Theta0), 0]];
         
Blat = [[Yda, Ydr];...
        [Lda, Ldr];...
        [Nda, Ndr];...
        [0, 0]];
        
% full model
A = [[Alon, zeros(4,4)]; [zeros(4,4), Alat]];
B = [[Blon, zeros(4,2)]; [zeros(4,2), Blat]];


%% Numeric Substitution
% config 1 from paper
% units are imperial
vXu=-0.008429; vXw=0.08624; vXq=0; vZu=-0.1241; vZw=-1.43; vZq=0;
vMu=0.000237; vMw=-0.02351; vMq=-1.652; vXde=0.07138; vXdt=0.002289; 
vZde=-1.249; vZdt=-0.001053; vMde=-0.192; vMdt = -3.826e-5;
vYv=-0.1698; vYp=0.8673; vYr=0; vLv=-0.01918; vLp=-2.278; vLr=0.8487; 
vNv=0.005268; vNp=-0.2258; vNr=-0.2719; vYda=-0.0132; vYdr=0.3073; 
vLda=-0.1623; vLdr=0.03301; vNda=-0.01127; vNdr=-0.03732; vg=-32.1740;
% this is the aircraft trim! 
vW0=0; vU0=0; vTheta0=0.03;


% Numeric Model
% FIXME: this is ugly
% idk how to make this better in matlab. In python, I could create a dict and
% correspond the symbolic variables to concrete values
An = double(subs(A, [Xu, Xw, Xq, Zu, Zw, Zq, Mu, Mw, Mq, Xde, Xdt, Zde,... 
            Zdt, Mde, Mdt, Yv, Yp, Yr, Lv, Lp, Lr, Nv, Np, Nr,...
            Yda, Ydr, Lda, Ldr, Nda, Ndr, W0, U0, Theta0, g],... 
            [vXu, vXw, vXq, vZu, vZw, vZq, vMu, vMw, vMq, vXde, vXdt, vZde,... 
            vZdt, vMde, vMdt, vYv, vYp, vYr, vLv, vLp, vLr, vNv, vNp, vNr,...
            vYda, vYdr, vLda, vLdr, vNda, vNdr, vW0, vU0, vTheta0, vg]));
            
Bn = double(subs(B, [Xu, Xw, Xq, Zu, Zw, Zq, Mu, Mw, Mq, Xde, Xdt, Zde,... 
            Zdt, Mde, Mdt, Yv, Yp, Yr, Lv, Lp, Lr, Nv, Np, Nr,...
            Yda, Ydr, Lda, Ldr, Nda, Ndr, W0, U0, Theta0, g],... 
            [vXu, vXw, vXq, vZu, vZw, vZq, vMu, vMw, vMq, vXde, vXdt, vZde,... 
            vZdt, vMde, vMdt, vYv, vYp, vYr, vLv, vLp, vLr, vNv, vNp, vNr,...
            vYda, vYdr, vLda, vLdr, vNda, vNdr, vW0, vU0, vTheta0, vg]));

            
%% Create State Space Model
learjet_sys = ss(An, Bn);