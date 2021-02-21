function PDOT = PDOT(P3,P1) % PDOT= rate of change of power
if (P1>=50.0) % P3= actual power, P1= power command
    if(P3>=50.0){
        T=5.0
        P2=P1
        }
    else{
        P2=60.0;
        T=RTAU(P2-P3);
        }
    end
else
    if (P3>= 50.0){
        T=5.0
        P2=40.0
        }
    else{
        P2=P1
        T=RTAU(P2-P3)
        }
    end
end
PDOT=T*(P2-P3);
return
end