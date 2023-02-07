import pvaccess as pva
import numpy as np
import time
import threading

import subprocess

from txmoptics import log
from epics import PV
import re

EPSILON = .001

def isfloat(x):
    try:
        a = float(x)
    except (TypeError, ValueError):
        return False
    else:
        return x.find('.')!=-1
    
class TXMOptics():
    """ Class for controlling TXM optics via EPICS

        Parameters
        ----------
        args : dict
            Dictionary of pv variables.
    """

    def __init__(self, pv_files, macros):

        # init pvs
        self.config_pvs = {}
        self.control_pvs = {}
        self.pv_prefixes = {}

        if not isinstance(pv_files, list):
            pv_files = [pv_files]
        for pv_file in pv_files:
            self.read_pv_file(pv_file, macros)
        self.show_pvs()

        prefix = self.pv_prefixes['CRLRelays']
        self.control_pvs['CRLRelaysY0'] = PV(prefix + 'oY0')
        self.control_pvs['CRLRelaysY1'] = PV(prefix + 'oY1')
        self.control_pvs['CRLRelaysY2'] = PV(prefix + 'oY2')
        self.control_pvs['CRLRelaysY3'] = PV(prefix + 'oY3')
        self.control_pvs['CRLRelaysY4'] = PV(prefix + 'oY4')
        self.control_pvs['CRLRelaysY5'] = PV(prefix + 'oY5')
        self.control_pvs['CRLRelaysY6'] = PV(prefix + 'oY6')
        self.control_pvs['CRLRelaysY7'] = PV(prefix + 'oY7')

        prefix = self.pv_prefixes['ValvesPLC']
        self.control_pvs['VPLCHighPressureOn'] = PV(prefix + 'oC23')
        self.control_pvs['VPLCHighPressureOff'] = PV(prefix + 'oC33')
        self.control_pvs['VPLCHighPressureStatus'] = PV(prefix + 'C3')
        self.control_pvs['VPLCLowPressureXOn'] = PV(prefix + 'oC22')
        self.control_pvs['VPLCLowPressureXOff'] = PV(prefix + 'oC32')
        self.control_pvs['VPLCLowPressureXStatus'] = PV(prefix + 'oC2')
        self.control_pvs['VPLCLowPressureYOn'] = PV(prefix + 'oC21')
        self.control_pvs['VPLCLowPressureYOff'] = PV(prefix + 'oC31')
        self.control_pvs['VPLCLowPressureYStatus'] = PV(prefix + 'oC1')
        self.control_pvs['VPLCHeFlow'] = PV(prefix + 'ao1')

        prefix = self.pv_prefixes['Shaker']
        self.control_pvs['ShakerRun'] = PV(prefix + 'run')
        self.control_pvs['ShakerFrequency'] = PV(prefix + 'frequency')
        self.control_pvs['ShakerTimePerPoint'] = PV(prefix + 'timePerPoint')
        self.control_pvs['ShakerNumPoints'] = PV(prefix + 'numPoints')
        self.control_pvs['ShakerAAmpMuliplyer'] = PV(prefix + 'A:ampMult')
        self.control_pvs['ShakerAAmpOffset'] = PV(prefix + 'A:ampOffset')
        self.control_pvs['ShakerAPhaseShift'] = PV(prefix + 'A:phaseShift')
        self.control_pvs['ShakerBAmpMuliplyer'] = PV(prefix + 'B:ampMult')
        self.control_pvs['ShakerBAmpOffset'] = PV(prefix + 'B:ampOffset')
        self.control_pvs['ShakerBFreqMult'] = PV(prefix + 'B:freqMult')

        prefix = self.pv_prefixes['BPM']
        self.control_pvs['BPMHSetPoint'] = PV(prefix + 'fb4.VAL')
        self.control_pvs['BPMHReadback'] = PV(prefix + 'fb4.CVAL')
        self.control_pvs['BPMHFeedback'] = PV(prefix + 'fb4.FBON')
        self.control_pvs['BPMHUpdateRate'] = PV(prefix + 'fb4.SCAN')
        self.control_pvs['BPMHKP'] = PV(prefix + 'fb4.KP')
        self.control_pvs['BPMHKI'] = PV(prefix + 'fb4.KI')
        self.control_pvs['BPMHKD'] = PV(prefix + 'fb4.KD')
        self.control_pvs['BPMHI'] = PV(prefix + 'fb4.I')
        self.control_pvs['BPMHLowLimit'] = PV(prefix + 'fb4.DRVL')
        self.control_pvs['BPMHHighLimit'] = PV(prefix + 'fb4.DRVH')
        self.control_pvs['BPMVSetPoint'] = PV(prefix + 'fb3.VAL')
        self.control_pvs['BPMVReadback'] = PV(prefix + 'fb3.CVAL')
        self.control_pvs['BPMVFeedback'] = PV(prefix + 'fb3.FBON')
        self.control_pvs['BPMVUpdateRate'] = PV(prefix + 'fb3.SCAN')
        self.control_pvs['BPMVKP'] = PV(prefix + 'fb3.KP')
        self.control_pvs['BPMVKI'] = PV(prefix + 'fb3.KI')
        self.control_pvs['BPMVKD'] = PV(prefix + 'fb3.KD')
        self.control_pvs['BPMVI'] = PV(prefix + 'fb3.I')
        self.control_pvs['BPMVLowLimit'] = PV(prefix + 'fb3.DRVL')
        self.control_pvs['BPMVHighLimit'] = PV(prefix + 'fb3.DRVH')

        prefix = self.pv_prefixes['Camera']
        self.control_pvs['CamAcquireTime'] = PV(prefix + 'cam1:AcquireTime')
        self.control_pvs['CamAcquire'] = PV(prefix + 'cam1:Acquire')
        self.control_pvs['CamTrans1Type'] = PV(prefix + 'Trans1:Type')        
        self.control_pvs['CamArraySizeXRBV']        = PV(prefix + 'cam1:ArraySizeX_RBV')
        self.control_pvs['CamArraySizeYRBV']        = PV(prefix + 'cam1:ArraySizeY_RBV')
        self.control_pvs['CamMinX']        = PV(prefix + 'cam1:MinX')
        self.control_pvs['CamMinY']        = PV(prefix + 'cam1:MinY')
        self.control_pvs['CamMinXRBV']        = PV(prefix + 'cam1:MinX_RBV')
        self.control_pvs['CamMinYRBV']        = PV(prefix + 'cam1:MinY_RBV')
        self.control_pvs['CamSizeX']        = PV(prefix + 'cam1:SizeX')
        self.control_pvs['CamSizeY']        = PV(prefix + 'cam1:SizeY')
        self.control_pvs['CamSizeXRBV']        = PV(prefix + 'cam1:SizeX_RBV')
        self.control_pvs['CamSizeYRBV']        = PV(prefix + 'cam1:SizeY_RBV')
        self.control_pvs['CamMaxSizeXRBV']        = PV(prefix + 'cam1:MaxSizeX_RBV')
        self.control_pvs['CamMaxSizeYRBV']        = PV(prefix + 'cam1:MaxSizeY_RBV')
        
        self.control_pvs['OPEnableCallbacks'] = PV(prefix + 'Over1:EnableCallbacks')
        self.control_pvs['OP1Use']            = PV(prefix + 'Over1:1:Use')        
        self.control_pvs['OP1CenterX']        = PV(prefix + 'Over1:1:CenterX')        
        self.control_pvs['OP1CenterY']        = PV(prefix + 'Over1:1:CenterY')        
        self.control_pvs['OP2Use']            = PV(prefix + 'Over1:2:Use')        
        self.control_pvs['OP2CenterX']        = PV(prefix + 'Over1:2:CenterX')        
        self.control_pvs['OP2CenterY']        = PV(prefix + 'Over1:2:CenterY')    


        self.control_pvs['EnergyMonochromator'] = PV('32ida:BraggEAO.VAL')
                
        self.epics_pvs = {**self.config_pvs, **self.control_pvs}

        for epics_pv in ('MoveCRLIn', 'MoveCRLOut', 'MovePhaseRingIn', 'MovePhaseRingOut', 'MoveDiffuserIn',
                         'MoveDiffuserOut', 'MoveBeamstopIn', 'MoveBeamstopOut', 'MovePinholeIn', 'MovePinholeOut',
                         'MoveCondenserIn', 'MoveCondenserOut', 'MoveZonePlateIn', 'MoveZonePlateOut', 'MoveFurnaceIn', 'MoveFurnaceOut',
                         'MoveAllIn', 'MoveAllOut', 'AllStop', 'SaveAllPVs', 'LoadAllPVs', 'CrossSelect', 'EnergySet', 'Crop',
                         'EnergyArbitrarySet', 'EnergyMoveSet'):
            self.epics_pvs[epics_pv].put(0)
            self.epics_pvs[epics_pv].add_callback(self.pv_callback)
        for epics_pv in ('ShutterBClose', 'ShutterBStatus'):
            self.epics_pvs[epics_pv].add_callback(self.pv_callback)            
        self.epics_pvs['EnergyBusy'].put(0,wait=True)
        
        # Start the watchdog timer thread
        thread = threading.Thread(target=self.reset_watchdog, args=(), daemon=True)
        thread.start()
        
        log.setup_custom_logger("./txmoptics.log")

    def read_pv_file(self, pv_file_name, macros):
        """Reads a file containing a list of EPICS PVs to be used by txmOptics.


        Parameters
        ----------
        pv_file_name : str
          Name of the file to read
        macros: dict
          Dictionary of macro substitution to perform when reading the file
        """

        pv_file = open(pv_file_name)
        lines = pv_file.read()
        pv_file.close()
        lines = lines.splitlines()
        for line in lines:
            is_config_pv = True
            if line.find('#controlPV') != -1:
                line = line.replace('#controlPV', '')
                is_config_pv = False
            line = line.lstrip()
            # Skip lines starting with #
            if line.startswith('#'):
                continue
            # Skip blank lines
            if line == '':
                continue
            pvname = line
            # Do macro substitution on the pvName
            for key in macros:
                pvname = pvname.replace(key, macros[key])
            # Replace macros in dictionary key with nothing
            dictentry = line
            for key in macros:
                dictentry = dictentry.replace(key, '')

            epics_pv = PV(pvname)

            if is_config_pv:
                self.config_pvs[dictentry] = epics_pv
            else:
                self.control_pvs[dictentry] = epics_pv
            # if dictentry.find('PVAPName') != -1:
            #     pvname = epics_pv.value
            #     key = dictentry.replace('PVAPName', '')
            #     self.control_pvs[key] = PV(pvname)
            if dictentry.find('PVName') != -1:
                pvname = epics_pv.value
                key = dictentry.replace('PVName', '')
                if (pvname != ''): 
                    print(pvname, key)
                    self.control_pvs[key] = PV(pvname)
            if dictentry.find('PVPrefix') != -1:
                pvprefix = epics_pv.value
                key = dictentry.replace('PVPrefix', '')
                self.pv_prefixes[key] = pvprefix

    def show_pvs(self):
        """Prints the current values of all EPICS PVs in use.

        The values are printed in three sections:

        - config_pvs : The PVs that are part of the scan configuration and
          are saved by save_configuration()

        - control_pvs : The PVs that are used for EPICS control and status,
          but are not saved by save_configuration()

        - pv_prefixes : The prefixes for PVs that are used for the areaDetector camera,
          file plugin, etc.
        """

        print('configPVS:')
        for config_pv in self.config_pvs:
            print(config_pv, ':', self.config_pvs[config_pv].get(as_string=True))

        print('')
        print('controlPVS:')
        for control_pv in self.control_pvs:
            print(control_pv, ':', self.control_pvs[control_pv].get(as_string=True))

        print('')
        print('pv_prefixes:')
        for pv_prefix in self.pv_prefixes:
            print(pv_prefix, ':', self.pv_prefixes[pv_prefix])

    def pv_callback(self, pvname=None, value=None, char_value=None, **kw):
        """Callback function that is called by pyEpics when certain EPICS PVs are changed
        """

        log.debug('pv_callback pvName=%s, value=%s, char_value=%s', pvname, value, char_value)
        if (pvname.find('MoveCRLIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_crl_in, args=())
            thread.start()
        elif (pvname.find('MoveCRLOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_crl_out, args=())
            thread.start()
        elif (pvname.find('MovePhaseRingIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_phasering_in, args=())
            thread.start()
        elif (pvname.find('MovePhaseRingOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_phasering_out, args=())
            thread.start()
        elif (pvname.find('MoveDiffuserIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_diffuser_in, args=())
            thread.start()
        elif (pvname.find('MoveDiffuserOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_diffuser_out, args=())
            thread.start()
        elif (pvname.find('MoveBeamstopIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_beamstop_in, args=())
            thread.start()
        elif (pvname.find('MoveBeamstopOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_beamstop_out, args=())
            thread.start()
        elif (pvname.find('MovePinholeIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_pinhole_in, args=())
            thread.start()
        elif (pvname.find('MovePinholeOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_pinhole_out, args=())
            thread.start()
        elif (pvname.find('MoveCondenserIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_condenser_in, args=())
            thread.start()
        elif (pvname.find('MoveCondenserOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_condenser_out, args=())
            thread.start()
        elif (pvname.find('MoveZonePlateIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_zoneplate_in, args=())
            thread.start()
        elif (pvname.find('MoveZonePlateOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_zoneplate_out, args=())
            thread.start()
        elif (pvname.find('MoveFurnaceIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_furnace_in, args=())
            thread.start()
        elif (pvname.find('MoveFurnaceOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_furnace_out, args=())
            thread.start()            
        elif (pvname.find('MoveAllIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_all_in, args=())
            thread.start()
        elif (pvname.find('MoveAllOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_all_out, args=())
            thread.start()
        elif (pvname.find('AllStop') != -1) and (value == 1):
            thread = threading.Thread(target=self.all_stop, args=())
            thread.start()       
        elif (pvname.find('SaveAllPVs') != -1) and (value == 1):
            thread = threading.Thread(target=self.save_all_pvs, args=())
            thread.start()       
        elif (pvname.find('LoadAllPVs') != -1) and (value == 1):
            thread = threading.Thread(target=self.load_all_pvs, args=())
            thread.start()       
        elif (pvname.find('CrossSelect') != -1) and ((value == 0) or (value == 1)):
            thread = threading.Thread(target=self.cross_select, args=())
            thread.start()
        elif (pvname.find('B:Close') != -1) and (value == 1):
            thread = threading.Thread(target=self.shutter_b_close, args=())
            thread.start()            
        elif (pvname.find('STA_B') != -1) and (value == 0):
            thread = threading.Thread(target=self.shutter_b_status, args=())
            thread.start()            
        elif (pvname.find('EnergySet') != -1) and (value == 1):
            thread = threading.Thread(target=self.energy_change, args=())
            thread.start()            
        elif (pvname.find('EnergyArbitrarySet') != -1) and (value == 1):
            thread = threading.Thread(target=self.energy_arbitrary_change, args=())
            thread.start()       
        elif (pvname.find('EnergyMoveSet') != -1) and (value == 1):
            thread = threading.Thread(target=self.energy_move_change, args=())
            thread.start()       
        elif (pvname.find('Crop') != -1) and (value ==1):
            thread = threading.Thread(target=self.crop_detector, args=())
            thread.start()            
        

    def move_crl_in(self):
        """Moves the crl in.
        """
        for k in range(7):
            if(self.epics_pvs['CRLRelaysY'+str(k)+'InOutUse'].value):
                self.control_pvs['CRLRelaysY'+str(k)].put(1, wait=True, timeout=1)

        self.epics_pvs['MoveCRLIn'].put('Done')

    def move_crl_out(self):
        """Moves the crl out.
        """
        for k in range(7):
            if(self.epics_pvs['CRLRelaysY'+str(k)+'InOutUse'].value):
                self.control_pvs['CRLRelaysY'+str(k)].put(0, wait=True, timeout=1)
        self.epics_pvs['MoveCRLOut'].put('Done')

    def move_diffuser_in(self):
        """Moves the diffuser in.
        """
        if(self.epics_pvs['DiffuserInOutUse'].value):
            position = self.epics_pvs['DiffuserInX'].value
            self.epics_pvs['DiffuserX'].put(position, wait=True)

        self.epics_pvs['MoveDiffuserIn'].put('Done')

    def move_diffuser_out(self):
        """Moves the diffuser out.
        """
        if(self.epics_pvs['DiffuserInOutUse'].value):
            position = self.epics_pvs['DiffuserOutX'].value
            self.epics_pvs['DiffuserX'].put(position, wait=True)

        self.epics_pvs['MoveDiffuserOut'].put('Done')

    def move_beamstop_in(self):
        """Moves the beamstop in.
        """
        if(self.epics_pvs['BeamstopInOutUse'].value):
            position = self.epics_pvs['BeamstopInY'].value
            self.epics_pvs['BeamstopY'].put(position, wait=True)

        self.epics_pvs['MoveBeamstopIn'].put('Done')

    def move_beamstop_out(self):
        """Moves the beamstop out.
        """
        if(self.epics_pvs['BeamstopInOutUse'].value):
            position = self.epics_pvs['BeamstopOutY'].value
            self.epics_pvs['BeamstopY'].put(position, wait=True)

        self.epics_pvs['MoveBeamstopOut'].put('Done')

    def move_pinhole_in(self):
        """Moves the pinhole in.
        """
        if(self.epics_pvs['PinholeInOutUse'].value):
            position = self.epics_pvs['PinholeInY'].value
            self.epics_pvs['PinholeY'].put(position, wait=True)

        self.epics_pvs['MovePinholeIn'].put('Done')

    def move_pinhole_out(self):
        """Moves the pinhole out.
        """
        if(self.epics_pvs['PinholeInOutUse'].value):
            position = self.epics_pvs['PinholeOutY'].value
            self.epics_pvs['PinholeY'].put(position, wait=True)

        self.epics_pvs['MovePinholeOut'].put('Done')

    def move_condenser_in(self):
        """Moves the condenser in.
        """
        if(self.epics_pvs['CondenserInOutUse'].value):
            position = self.epics_pvs['CondenserInY'].value
            self.epics_pvs['CondenserY'].put(position, wait=True)

        self.epics_pvs['MoveCondenserIn'].put('Done')

    def move_condenser_out(self):
        """Moves the condenser out.
        """
        if(self.epics_pvs['CondenserInOutUse'].value):
            position = self.epics_pvs['CondenserOutY'].value
            self.epics_pvs['CondenserY'].put(position, wait=True)

        self.epics_pvs['MoveCondenserOut'].put('Done')

    def move_zoneplate_in(self):
        """Moves the zone plate in.
        """
        if(self.epics_pvs['ZonePlateInOutUse'].value):
            position = self.epics_pvs['ZonePlateInY'].value
            self.epics_pvs['ZonePlateY'].put(position, wait=True)

        self.epics_pvs['MoveZonePlateIn'].put('Done')

    def move_zoneplate_out(self):
        """Moves the zone plate out.
        """
        if(self.epics_pvs['ZonePlateInOutUse'].value):
            position = self.epics_pvs['ZonePlateOutY'].value
            self.epics_pvs['ZonePlateY'].put(position, wait=True)

        self.epics_pvs['MoveZonePlateOut'].put('Done')

    def move_phasering_in(self):
        """Moves the phase ring in.
        """
        if(self.epics_pvs['PhaseRingInOutUse'].value):
            position = self.epics_pvs['PhaseRingInY'].value
            self.epics_pvs['PhaseRingY'].put(position, wait=True)

        self.epics_pvs['MovePhaseRingIn'].put('Done')

    def move_phasering_out(self):
        """Moves the phase ring out.
        """
        if(self.epics_pvs['PhaseRingInOutUse'].value):
            position = self.epics_pvs['PhaseRingOutY'].value
            self.epics_pvs['PhaseRingY'].put(position, wait=True)

        self.epics_pvs['MovePhaseRingOut'].put('Done')

    def move_furnace_in(self):
        """Moves the furnace in.
        """
        if(self.epics_pvs['FurnaceInOutUse'].value):
            position = self.epics_pvs['FurnaceInY'].value
            self.epics_pvs['FurnaceY'].put(position, wait=True)

        self.epics_pvs['MoveFurnaceIn'].put('Done')

    def move_furnace_out(self):
        """Moves the furnace out.
        """
        if(self.epics_pvs['FurnaceInOutUse'].value):
            position = self.epics_pvs['FurnaceOutY'].value
            self.epics_pvs['FurnaceY'].put(position, wait=True)

        self.epics_pvs['MoveFurnaceOut'].put('Done')

    def set_exposure_time_in(self):
        """Set exposure time in.
        """
        if(self.epics_pvs['ExposureTimeInOutUse'].value):
            exposure_time = self.epics_pvs['ExposureTimeIn'].value
        self.epics_pvs['CamAcquireTime'].put(exposure_time, wait=True, timeout=10.0)

    def set_exposure_time_out(self):
        """Set exposure time out.
        """
        if(self.epics_pvs['ExposureTimeInOutUse'].value):
            exposure_time = self.epics_pvs['ExposureTimeOut'].value
        self.epics_pvs['CamAcquireTime'].put(exposure_time, wait=True, timeout=10.0)

    def set_bpm_in(self):
        """
        Set BPM readback value in
        """
        if(self.epics_pvs['BPMSetPointInOutUse'].value):
            positionv = self.epics_pvs['BPMVSetPointIn'].value
            positionh = self.epics_pvs['BPMHSetPointIn'].value
            print(self.epics_pvs['BPMVSetPoint'].get(), positionv)
            self.epics_pvs['BPMVSetPoint'].put(positionv, wait=True)
            self.epics_pvs['BPMHSetPoint'].put(positionh, wait=True)            
    
    def set_bpm_out(self):
        """
        Set BPM readback value out
        """
        if(self.epics_pvs['BPMSetPointInOutUse'].value):
            positionv = self.epics_pvs['BPMVSetPointOut'].value
            positionh = self.epics_pvs['BPMHSetPointOut'].value
            self.epics_pvs['BPMVSetPoint'].put(positionv, wait=True)
            self.epics_pvs['BPMHSetPoint'].put(positionh, wait=True)            

    def transform_image_in(self):
        """
        Transform image in 
        """
        self.epics_pvs['CamTrans1Type'].put(2, wait=True) # None
        
    def transform_image_out(self):
        """
        Transform image out 
        """
        self.epics_pvs['CamTrans1Type'].put(0, wait=True) # Rotate180
                
    def move_all_in(self):
        """Moves all in
        """
        funcs = [self.move_crl_in,
                 self.set_bpm_in,
                 self.move_phasering_in,
                 self.move_diffuser_in,
                 self.move_beamstop_in,
                 self.move_pinhole_in,
                 self.move_condenser_in,
                 self.move_zoneplate_in,
                 self.move_furnace_in,
                 self.set_exposure_time_in,
                 self.transform_image_in,
                 ]
        threads = [threading.Thread(target=f, args=()) for f in funcs]
        [t.start() for t in threads]
        [t.join() for t in threads]

        self.epics_pvs['MoveAllIn'].put('Done')

    def move_all_out(self):
        """Moves all out
        """
        funcs = [self.move_crl_out,
                 self.set_bpm_out,
                 self.move_phasering_out,
                 self.move_diffuser_out,
                 self.move_beamstop_out,
                 self.move_pinhole_out,
                 self.move_condenser_out,
                 self.move_zoneplate_out,
                 self.move_furnace_out,
                 self.set_exposure_time_out,
                 self.transform_image_out,
                 ]
        threads = [threading.Thread(target=f, args=()) for f in funcs]
        [t.start() for t in threads]
        [t.join() for t in threads]

        self.epics_pvs['MoveAllOut'].put('Done')
        
    def all_stop(self):
        """Stop all iocs motors
        """     
        iocs = [self.pv_prefixes['IOC'+str(k)] for k in range(5)]
        print(iocs)
        allstop_pvs = [PV(ioc+'allstop') for ioc in iocs]                        
        [pv.put(1,wait=True) for pv in allstop_pvs]
        self.epics_pvs['AllStop'].put(0,wait=True)
    
    def save_all_pvs(self):
        """Save all PVs from txm_main.adl screen to a file
        """
        if(self.epics_pvs['LoadAllPVs'].get()==1):
            self.epics_pvs['SaveAllPVs'].put(0,wait=True)       
            return
        file_name = self.epics_pvs['FileAllPVs'].get()
        # read prefixes
        # with open('/home/beams/USERTXM/epics/synApps/support/txmoptics/iocBoot/iocTXMOptics/start_medm','r') as fid:    
        #     prefixes = re.findall(r"-macro \"(.*)\"", fid.read())[0].split(', ')
        # # take all replacements
        # repl = []
        # for k in prefixes:
        #     repl.append(k.split('='))
        # # read adl file
        with open('/home/beams/USERTXM/epics/synApps/support/txmoptics/txmOpticsApp/op/adl/txm_main.adl','r') as fid:    
            s = fid.read()
        # # replace in adl file
        # for k in repl:
        #     s = s.replace('$('+k[0]+')',k[1])
        # take pvs
        pvs = []
        pvs = re.findall(r"chan=\"(.*?)\"", s)
        
        # save values to a txt file 
        try:
            with open(file_name,'w') as fid:
                energy = self.epics_pvs['EnergyMonochromator'].get()
                fid.write('energy '+ str(energy) +'\n')                
                for k in pvs:
                    if k.find('.VAL')!=-1:
                        p = PV(k)
                        #time.sleep(0.1)
                        val = p.get(as_string=True)
                        if(val is not None and isfloat(val)):
                            print(k,val)                        
                            fid.write(k[:-4]+' '+val+'\n')
        except:
            log.error('File %s cannot be created', file_name)
        self.epics_pvs['SaveAllPVs'].put(0,wait=True)        
    
    def load_all_pvs(self):
        """Load all PVs to txm_main.adl screen to a file
        """
        if(self.epics_pvs['SaveAllPVs'].get()==1):
            self.epics_pvs['LoadAllPVs'].put(0,wait=True)        
            return
        file_name = self.epics_pvs['FileAllPVs'].get()        
        try:
            with open(file_name,'r') as fid:
                for pv_val in fid.readlines():
                    pv, val = pv_val[:-1].split(' ')
                    print(pv,val)
                    try:
                        PV(pv).put(val,wait=True)
                    except:
                        pass
        except:
            log.error('File %s does not exist or corrupted', file_name)
        self.epics_pvs['LoadAllPVs'].put(0,wait=True)

    def cross_select(self):
        """Plot the cross in imageJ.
        """
    

        if (self.epics_pvs['CrossSelect'].get() == 0):
            sizex = int(self.epics_pvs['CamSizeXRBV'].get())
            sizey = int(self.epics_pvs['CamSizeYRBV'].get())
            self.epics_pvs['OP1CenterX'].put(sizex//2)
            self.epics_pvs['OP1CenterY'].put(sizey//2)
            self.control_pvs['OP1Use'].put(1)
            self.epics_pvs['OP2CenterX'].put(sizex//2)
            self.epics_pvs['OP2CenterY'].put(sizey//2)
            self.control_pvs['OP2Use'].put(1)
            log.info('Cross at %d %d is enable' % (sizex//2,sizey//2))
        else:
            self.control_pvs['OP1Use'].put(0)
            self.control_pvs['OP2Use'].put(0)
            log.info('Cross is disabled')

 
    def shutter_b_close(self):        
        if (self.epics_pvs['ShutterCallback'].get() == 0):
            log.info('Stop BPM')
            self.epics_pvs['BPMVFeedback'].put(0)
            self.epics_pvs['BPMHFeedback'].put(0)  
        
    def shutter_b_status(self):
        if (self.epics_pvs['ShutterCallback'].get() == 0):
            log.info('Start BPM')
            self.epics_pvs['BPMVFeedback'].put(1)
            self.epics_pvs['BPMHFeedback'].put(1)  

    def energy_change(self):
        if self.epics_pvs['EnergyBusy'].get() == 0:
            self.epics_pvs['EnergyBusy'].put(1)
            self.epics_pvs['BPMVFeedback'].put(0)
            self.epics_pvs['BPMHFeedback'].put(0)      
            energy = float(self.epics_pvs["Energy"].get())
            log.info("TxmOptics: change energy to %.2f",energy)
            
            log.info('move monochromator')            
            self.epics_pvs['DCMputEnergy'].put(energy)
            log.info('move undulator')
            self.epics_pvs['GAPputEnergy'].put(energy+0.15)
            self.wait_pv(self.epics_pvs['DCMputEnergyRBV'],energy)
            self.wait_pv(self.epics_pvs['GAPputEnergyRBV'],energy+0.15)
            
            time.sleep(1)# possible backlash/stabilization, more??
            if self.epics_pvs['EnergyUseCalibration'].get(as_string=True) == 'Yes':                
                try:
                    # read pvs for 2 energies
                    pvs1, pvs2, vals1, vals2 = [],[],[],[]
                    with open(self.epics_pvs['EnergyCalibrationFileOne'].get()) as fid:
                        for pv_val in fid.readlines():
                            pv, val = pv_val[:-1].split(' ')
                            pvs1.append(pv)
                            vals1.append(float(val))
                    with open(self.epics_pvs['EnergyCalibrationFileTwo'].get()) as fid:
                        for pv_val in fid.readlines():
                            pv, val = pv_val[:-1].split(' ')
                            pvs2.append(pv)
                            vals2.append(float(val))                    
                    
                    for k in range(len(pvs1)):
                        if(pvs1[k]!=pvs2[k]):                            
                            raise Exception()                            
                    if(np.abs(vals2[0]-vals1[0])<0.001):            
                        raise Exception()           
                    vals = []                     
                    for k in range(len(pvs1)):
                        vals.append(vals1[k]+(energy-vals1[0])*(vals2[k]-vals1[k])/(vals2[0]-vals1[0]))               
                    # set new pvs  
                    for k in range(1,len(pvs1)):# skip energy line                        
                        if pvs1[k]==self.epics_pvs['DetectorZ'].pvname:                            
                            log.info('old Detector Z %3.3f', self.epics_pvs['DetectorZ'].get())
                            self.epics_pvs['DetectorZ'].put(vals[k],wait=True)                                                        
                            log.info('new Detector Z %3.3f', self.epics_pvs['DetectorZ'].get())
                        if pvs1[k]==self.epics_pvs['ZonePlateZ'].pvname:                            
                            log.info('old Zone plate Z %3.3f', self.epics_pvs['ZonePlateZ'].get())
                            self.epics_pvs['ZonePlateZ'].put(vals[k],wait=True)                                                        
                            log.info('new Zone plate Z %3.3f', self.epics_pvs['ZonePlateZ'].get())
                        # if pvs1[k]==self.epics_pvs['ZonePlateX'].pvname:                            
                        #     log.info('old Zone plate X %3.3f', self.epics_pvs['ZonePlateX'].get())
                        #     self.epics_pvs['ZonePlateX'].put(vals[k],wait=True)                                                        
                        #     log.info('new Zone plate X %3.3f', self.epics_pvs['ZonePlateX'].get())
                        #maybe  y too..                        
                except:
                    log.error('Calibration files are wrong.')
            self.epics_pvs['BPMVFeedback'].put(1)
            self.epics_pvs['BPMHFeedback'].put(1)                                  
            log.info('energy change is done')
            self.epics_pvs['EnergyBusy'].put(0)   
            self.epics_pvs['EnergySet'].put(0)      

    def energy_arbitrary_change(self):

        if self.epics_pvs['EnergyBusy'].get() == 1:
            return
            
        # self.epics_pvs['MCTStatus'].put('Changing energy')
        self.epics_pvs['EnergyBusy'].put(1)

        energy_choice_min = float(PV(self.epics_pvs["EnergyChoice"].pvname + '.ONST').get().split(' ')[1])
        # print(energy_choice_min)
        energy_choice_max = float(PV(self.epics_pvs["EnergyChoice"].pvname + '.TWST').get().split(' ')[1])
        # print(self.epics_pvs["EnergyChoice"].pvname + '.NIST')

        # print(energy_choice_min, energy_choice_max)
        self.epics_pvs['EnergyBusy'].put(0)

        if self.epics_pvs["EnergyCalibrationUse"].get(as_string=True) == "Pre-set":
            # self.epics_pvs['MCTStatus'].put('Changing energy: using presets')

            energy_choice_index = str(self.epics_pvs["EnergyChoice"].get())
            energy_choice       = self.epics_pvs["EnergyChoice"].get(as_string=True)
            energy_choice_list  = energy_choice.split(' ')
            log.info("txmOptics: energy choice = %s",energy_choice)
            if energy_choice_list[0] == 'Pink':
                command = 'energy pink --force'
            else: # Mono
                command = 'energy set --energy ' + energy_choice_list[1] + ' --force'
        else:
            energy_arbitrary = self.epics_pvs['EnergyArbitrary'].get()
            if  (energy_arbitrary >= energy_choice_min) & (energy_arbitrary < energy_choice_max):
                command = 'energy set --energy ' + str(self.epics_pvs['EnergyArbitrary'].get()) + ' --force'
                self.epics_pvs['EnergyInRange'].put(1)
            else:
                # self.epics_pvs['MCTStatus'].put('Error: energy out of range')
                self.epics_pvs['EnergyInRange'].put(0)
                time.sleep(2) # for testing
                # self.epics_pvs['MCTStatus'].put('Done')
                self.epics_pvs['EnergyBusy'].put(0)   
                self.epics_pvs['EnergyArbitrarySet'].put(0)
                return
        if (self.epics_pvs["EnergyTesting"].get()):
            command =  command + ' --testing' 
        
        log.error(command)
        # subprocess.Popen(command, shell=True)        

        time.sleep(5)
        log.info('txmOptics: waiting on motion to complete')
        # while True:
        #     time.sleep(.3)
        #     if PV('2bma:alldone').get() and PV('2bmb:alldone').get():
        #         break
        log.info('motion completed')

        time.sleep(2) # for testing
        # self.epics_pvs['MCTStatus'].put('Done')
        self.epics_pvs['EnergyBusy'].put(0)   
        self.epics_pvs['EnergyArbitrarySet'].put(0)   

    def energy_move_change(self):

        if self.epics_pvs['EnergyBusy'].get() == 1:
            return
            
        # self.epics_pvs['MCTStatus'].put('Changing energy move setting update')
        self.epics_pvs['EnergyBusy'].put(1)
        command = 'energy status'
        log.error(command)
        # subprocess.Popen(command, shell=True)     
        time.sleep(2) # for testing
        # self.epics_pvs['MCTStatus'].put('Done')
        self.epics_pvs['EnergyMoveSet'].put(0)   
        self.epics_pvs['EnergyBusy'].put(0)


    def crop_detector(self):
        """crop detector sizes"""
        state = self.epics_pvs['CamAcquire'].get()
        self.epics_pvs['CamAcquire'].put(0,wait=True)

        maxsizex = self.epics_pvs['CamMaxSizeXRBV'].get()
        self.epics_pvs['CamMinX'].put(0,wait=True)        
        
        maxsizey = self.epics_pvs['CamMaxSizeYRBV'].get()
        self.epics_pvs['CamMinY'].put(0,wait=True)        
        
        left = self.epics_pvs['CropLeft'].get()
        top = self.epics_pvs['CropBottom'].get()#flipped due to the ZP
        
        right = self.epics_pvs['CropRight'].get()        
        self.epics_pvs['CamSizeX'].put(maxsizex-left-right,wait=True)
        sizex = self.epics_pvs['CamSizeXRBV'].get()
        right = maxsizex - left - sizex
        self.epics_pvs['CropRight'].put(right,wait=True)

        bottom = self.epics_pvs['CropTop'].get()
        self.epics_pvs['CamSizeY'].put(maxsizey-top-bottom,wait=True)
        sizey = self.epics_pvs['CamSizeYRBV'].get()
        bottom = maxsizey - top - sizey
        self.epics_pvs['CropTop'].put(bottom,wait=True)

        self.epics_pvs['CamMinX'].put(left,wait=True)        
        left = self.epics_pvs['CamMinXRBV'].get()
        self.epics_pvs['CropLeft'].put(left,wait=True)

        self.epics_pvs['CamMinY'].put(top,wait=True)        
        top = self.epics_pvs['CamMinYRBV'].get()
        self.epics_pvs['CropBottom'].put(top,wait=True)                
        self.epics_pvs['CamAcquire'].put(1)  # need to take at least 1 frame
        
        self.cross_select()      
        self.epics_pvs['Crop'].put(0,wait=True)  

    def reset_watchdog(self):
        """Sets the watchdog timer to 5 every 3 seconds"""
        while True:
            self.epics_pvs['Watchdog'].put(5)
            time.sleep(3)  
    
    def wait_pv(self, epics_pv, wait_val, timeout=-1):
        """Wait on a pv to be a value until max_timeout (default forever)
           delay for pv to change
        """

        time.sleep(.01)
        start_time = time.time()
        while True:
            pv_val = epics_pv.get()
            if isinstance(pv_val, float):
                if abs(pv_val - wait_val) < EPSILON:
                    return True
            if pv_val != wait_val:
                if timeout > -1:
                    current_time = time.time()
                    diff_time = current_time - start_time
                    if diff_time >= timeout:
                        log.error('  *** wait_pv(%s, %d, %5.2f reached max timeout. Return False',
                                      epics_pv.pvname, wait_val, timeout)
                        return False
                time.sleep(.01)
            else:
                return True