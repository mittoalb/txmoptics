



for i in {1..10}; do
	for i in {1..10}; do
    		tomoscan single --tomoscan-prefix 32id:TomoScan:
		caput 32idbTXM:mcs:c2:m1 0.026
		#Set to zero
		caput 32idbTXM:mcs:c2:m1.SET 1
		caput 32idbTXM:mcs:c2:m1.VAL 0 
		caput 32idbTXM:mcs:c2:m1.SET 0
	done
	
	#Move of step
	caput 32idbTXM:mcs:c2:m2 0.036
	#Set back to zero
	caput 32idbTXM:mcs:c2:m1.SET 1
	caput 32idbTXM:mcs:c2:m1.VAL 0 
	caput 32idbTXM:mcs:c2:m1.SET 0
	#Set back to zero
	caput 32idbTXM:mcs:c2:m2 -0.26
	caput 32idbTXM:mcs:c2:m2.SET 1
	caput 32idbTXM:mcs:c2:m2.VAL 0 
	caput 32idbTXM:mcs:c2:m2.SET 0
done

