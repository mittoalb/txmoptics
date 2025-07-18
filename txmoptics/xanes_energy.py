import numpy as np
import sys
import os


emin = float(sys.argv[1])
emax = float(sys.argv[2])
npts = int(sys.argv[3])
outfile = sys.argv[4] if len(sys.argv) > 4 else '~/energies.npy'


energies = np.linspace(emin, emax, npts)
np.save(os.path.expanduser(outfile), energies)

print(f"Saved {npts} points from {emin} to {emax} keV in {outfile}")

