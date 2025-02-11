vars='P=32idcTXM:, P0=32id:, R0=TomoScanStep:, B0=tomoScanStep_32ID, P1=32id:, R1=TomoScan:, B1=tomoScan_32ID, P3=32id:, R3=TomoScanStream:, B3=tomoScanStream_32ID, P2=32idb:, BL=32ID, P4=32ida:, SK=32idcMC:, PG=32idcSP1:, R=cam1:, R2=Proc1:, P5=32id:, R5=TXMOptics:, P6=32id:, P7=32idcEXP:, P8=32idcUC8:, P9=32idc02:, P10=32idcTEMP:, P11=32idcSOFT:, R6=TomoScanStream:, B6=tomoScanStream_32ID, RO=obj, PID1=fb4, PID2=fb3, VALVES_PLC=32idPLC:C:, CRLRelays=32idPLC:B:, Contr2=xps:c1:, Contr5=nf:c0:, Contr6=ens:c1:, Contr7=nf:c0:, Contr8=mxv:c1:, Contr12=mcs:c2:, Contr13=mcs:c3:, Contr14=mcs:c0:, Contr15=nf:c1:, P12=32idc01:, S=uniblitz:, T=tc1, CT=heater, CV=out'

for k in $vars; do
    d=$(echo $k |sed 's/,//';)
    #echo $k |sed 's/,//')
    a=($(echo $d| tr "=" "\n" ))
    echo ${a[0]}
    v=\$\(${a[0]}\) 
    t=${a[1]} 
    echo $v $t   
    sed -i "s/$v/$t/g" ../../txmOpticsApp/op/adl/txm_main.adl
done
#a = ($(echo P=32idcTXM: | tr "=" "\n" ))