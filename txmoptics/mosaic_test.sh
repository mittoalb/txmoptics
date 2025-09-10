#!/bin/bash

# test_mosaic.sh - Test script that only prints the input parameters
# Used for testing the Python GUI parameter passing

echo "=== MOSAIC PARAMETER TEST ==="
echo "Script called with $# parameters"
echo ""

if [ $# -lt 4 ]; then
    echo "Error: Missing parameters!"
    echo "Usage: $0 <h_steps> <v_steps> <h_step_size> <v_step_size> [tomoscan_prefix]"
    echo "Example: $0 3 3 0.2 0.2 32id:TomoScan:"
    exit 1
fi

# Read and display all parameters
echo "Parameter 1 (H_STEPS): $1"
echo "Parameter 2 (V_STEPS): $2" 
echo "Parameter 3 (H_STEP_SIZE): $3"
echo "Parameter 4 (V_STEP_SIZE): $4"
echo "Parameter 5 (TOMOSCAN_PREFIX): ${5:-'(not provided - would use default)'}"

echo ""
echo "=== PARSED VALUES ==="
H_STEPS=$1
V_STEPS=$2
H_STEP_SIZE=$3
V_STEP_SIZE=$4
TOMOSCAN_PREFIX=${5:-"32id:TomoScan:"}

echo "Horizontal steps: $H_STEPS"
echo "Vertical steps: $V_STEPS"
echo "Horizontal step size: ${H_STEP_SIZE} mm"
echo "Vertical step size: ${V_STEP_SIZE} mm"
echo "Tomoscan prefix: $TOMOSCAN_PREFIX"
echo "Total scans would be: $((H_STEPS * V_STEPS))"

echo ""
echo "=== TEST COMPLETED SUCCESSFULLY ==="
echo "Parameters received and parsed correctly!"

# Simulate some processing time
sleep 2

echo "Test script finished - ready for real acquisition!"
