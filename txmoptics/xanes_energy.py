import numpy as np
import subprocess
import os

def caget(pv):
    try:
        result = subprocess.run(['caget', '-t', pv], stdout=subprocess.PIPE, check=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        print(f"[ERROR] Reading {pv}: {e}")
        exit(1)

# Read energy scan parameters
emin = float(caget("32id:TXMOptics:XanesStart"))
emax = float(caget("32id:TXMOptics:XanesEnd"))
steps = int(float(caget("32id:TXMOptics:XanesStep")))

# Read calibration file paths
params1 = caget("32id:TXMOptics:EnergyCalibrationFileOne")
params2 = caget("32id:TXMOptics:EnergyCalibrationFileTwo")


params1 = "/home/beams/USERTXM/epics/synApps/support/txmoptics/iocBoot/iocTXMOptics/" + params1
params2 = "/home/beams/USERTXM/epics/synApps/support/txmoptics/iocBoot/iocTXMOptics/" + params2

# Save energy array
outfile = os.path.expanduser("~/energies.npy")

npts = int((emax*1000 - emin*1000)/steps)+1
energies = np.linspace(emin, emax, npts)
np.save(outfile, energies)
print(f"[INFO] calculated energies:", energies)
print(f"[INFO] Saved {npts} points from {emin} to {emax} keV in {outfile}")
print(f"[INFO] Using calibration files:\n  - {params1}\n  - {params2}")

subprocess.run([
     "tomoscan", "energy",
     "--tomoscan-prefix", "32id:TomoScan:",
     "--file-params1", params1,
     "--file-params2", params2,
     "--file-energies", outfile
])

#Stop shaker and switch off the feedback
subprocess.run([
     "caput 32idbSoft:epidH:on off",
     "caput 32idbSoft:epidV:on off",
     "caput 32idbShaker:shaker:run Stop"
])

