#!/bin/bash
#Run a tomoscan energy scan

# Define variables
TAB_NAME="tomoScan XANES"
REMOTE_USER="usertxm"
REMOTE_HOST="txm4"
CONDA_ENV="tomoscan"
WORK_DIR="/home/beams/USERTXM/epics/synApps/support/tomoscan/iocBoot/iocTomoScan_32ID/"
CONDA_PATH="/home/beams/USERTXM/conda/anaconda/"

# Open a new tab in gnome-terminal, SSH into tomdet, activate conda, and run Python (without login shell)
gnome-terminal --tab --title="$TAB_NAME" -- bash -c "
    ssh -t ${REMOTE_USER}@${REMOTE_HOST} '
        bash -l -c \"cd ${WORK_DIR} && hostname &&\
        source ${CONDA_PATH}/etc/profile.d/conda.sh && \
        conda activate ${CONDA_ENV} && \
        
        XanesStart=$(caget -t 32id:TXMOptics:XanesStart)
        XanesEnd=$(caget -t 32id:TXMOptics:XanesEnd)
        XanesPoints=$(caget -t 32id:TXMOptics:XanesPoints)
        params1=$(caget -t 32id:TXMOptics:EnergyCalibrationFileOne)
        params2=$(caget -t 32id:TXMOptics:EnergyCalibrationFileTwo)
      
        python xanes_energy.py XanesStart XanesEnd XanesPoints
        tomoscan energy --tomoscan-prefix 32id:TomoScanStep: --file-params1 $params1 --file-params2 $params2 --file-energies ~/energies.npy 
    '
"

