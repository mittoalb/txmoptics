#!/bin/bash

# Run a tomoscan energy scan
TAB_NAME="tomoScan XANES"
REMOTE_USER="usertxm"
REMOTE_HOST="txm4"
CONDA_ENV="tomoscan"
WORK_DIR="/home/beams/USERTXM/epics/synApps/support/tomoscan/iocBoot/iocTomoScan_32ID/"
CONDA_PATH="/home/beams/USERTXM/conda/anaconda"
SCRIPT_NAME="/home/beams/USERTXM/epics/synApps/support/txmoptics/txmoptics/xanes_energy.py"

# Ensure DISPLAY is set properly even in SSH or headless environments

if [[ "$DISPLAY" =~ ^localhost:.* || "$DISPLAY" =~ ^:1[0-9]+ ]]; then
    export DISPLAY="$DISPLAY"
else
    for xsock in /tmp/.X11-unix/X*; do
        num="${xsock##*/X}"
        if [ -O "$xsock" ]; then
            export DISPLAY=":$num"
            break
        fi
    done
fi

# Allow the current user to use the X server
xhost +SI:localuser:$USER > /dev/null 2>&1


gnome-terminal --tab --title="$TAB_NAME" -- bash -c "
    ssh -t ${REMOTE_USER}@${REMOTE_HOST} '
        bash -l -c \"cd ${WORK_DIR} && hostname && \
        source ${CONDA_PATH}/etc/profile.d/conda.sh && \
        conda activate ${CONDA_ENV} && \
        python ${SCRIPT_NAME} && \
        python -i ; \"
    '
"
