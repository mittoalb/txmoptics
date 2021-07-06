import pvaccess as pva
import numpy as np
import queue
import time
import h5py
import threading
import signal

from txmoptics import util
from txmoptics import log
from epics import PV


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
        self.control_pvs['CamTrans1Type'] = PV(prefix + 'Trans1:Type')

        #Define PVs we will need for setting the user coordinates to zero
        phase_ring_pv_name = self.control_pvs['PhaseRingY'].pvname
        self.control_pvs['PhaseRingYSet']        = PV(phase_ring_pv_name + '.SET')
        diffuser_pv_name = self.control_pvs['DiffuserX'].pvname
        self.control_pvs['DiffuserXSet']        = PV(diffuser_pv_name + '.SET')
        beamstop_pv_name = self.control_pvs['BeamstopY'].pvname
        self.control_pvs['BeamstopYSet']        = PV(beamstop_pv_name + '.SET')
        pinhole_pv_name = self.control_pvs['PinholeY'].pvname
        self.control_pvs['PinholeYSet']        = PV(pinhole_pv_name + '.SET')
        condenser_pv_name = self.control_pvs['CondenserY'].pvname
        self.control_pvs['CondenserYSet']        = PV(condenser_pv_name + '.SET')
        zone_plate_pv_name = self.control_pvs['ZonePlateY'].pvname
        self.control_pvs['ZonePlateYSet']        = PV(zone_plate_pv_name + '.SET')
        sample_top_pv_name = self.control_pvs['SampleTopX'].pvname
        self.control_pvs['SampleTopXSet']        = PV(sample_top_pv_name + '.SET')

        # All Stop ioc PVs (--> add to the gui)
        iocs = ['32idcTXM:','32idcPLC:', '32idcEXP:', '32idb:', '32idcUC8:', '32idcMC:']
        self.allstop_pvs = [PV(ioc+'allstop') for ioc in iocs]
                
        self.epics_pvs = {**self.config_pvs, **self.control_pvs}

        for epics_pv in ('MoveCRLIn', 'MoveCRLOut', 'MovePhaseRingIn', 'MovePhaseRingOut', 'MoveDiffuserIn',
                         'MoveDiffuserOut', 'MoveBeamstopIn', 'MoveBeamstopOut', 'MovePinholeIn', 'MovePinholeOut',
                         'MoveCondenserIn', 'MoveCondenserOut', 'MoveZonePlateIn', 'MoveZonePlateOut',
                         'MoveAllIn', 'MoveAllOut', 'AllStop', 'SetAllToZero'):
            self.epics_pvs[epics_pv].add_callback(self.pv_callback)

        log.setup_custom_logger("./txmoptics.log")

    def read_pv_file(self, pv_file_name, macros):
        """Reads a file containing a list of EPICS PVs to be used by TXMOptics.


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
        elif (pvname.find('MoveSampleTopIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_sampletop_in, args=())
            thread.start()
        elif (pvname.find('MoveSampleTopOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_sampletop_out, args=())
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
        elif (pvname.find('SetPhaseRingToZero') != -1) and (value == 1):
            thread = threading.Thread(target=self.set_phasering_to_zero, args=())
            thread.start()
        elif (pvname.find('SetDiffuserToZero') != -1) and (value == 1):
            thread = threading.Thread(target=self.set_diffuser_to_zero, args=())
            thread.start()
        elif (pvname.find('SetBeamstopToZero') != -1) and (value == 1):
            thread = threading.Thread(target=self.set_beamstop_to_zero, args=())
            thread.start()
        elif (pvname.find('SetPinholeToZero') != -1) and (value == 1):
            thread = threading.Thread(target=self.set_pinhole_to_zero, args=())
            thread.start()
        elif (pvname.find('SetCondenserToZero') != -1) and (value == 1):
            thread = threading.Thread(target=self.set_condenser_to_zero, args=())
            thread.start()
        elif (pvname.find('SetZonePlateToZero') != -1) and (value == 1):
            thread = threading.Thread(target=self.set_zone_plate_to_zero, args=())
            thread.start()
        elif (pvname.find('SetSampleTopToZero') != -1) and (value == 1):
            thread = threading.Thread(target=self.set_sample_top_to_zero, args=())
            thread.start()
        elif (pvname.find('SetAllToZero') != -1) and (value == 1):
            thread = threading.Thread(target=self.set_all_to_zero, args=())
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

    def move_sampletop_in(self):
        """Moves sample top in.
        """
        if(self.epics_pvs['SampleTopInOutUse'].value):
            position = self.epics_pvs['SampleTopInX'].value
            self.epics_pvs['SampleTopX'].put(position, wait=True)

        self.epics_pvs['MoveSampleTopIn'].put('Done')

    def move_sampletop_out(self):
        """Moves sample top out.
        """
        if(self.epics_pvs['SampleTopInOutUse'].value):
            position = self.epics_pvs['SampleTopOutX'].value
            self.epics_pvs['SampleTopX'].put(position, wait=True)

        self.epics_pvs['MoveSampleTopOut'].put('Done')


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
        self.epics_pvs['CamTrans1Type'].put(2, wait=True) # Rot180
        
    
    def transform_image_out(self):
        """
        Transform image out 
        """
        self.epics_pvs['CamTrans1Type'].put(0, wait=True) # None
                
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
        [pv.put(1,wait=True) for pv in self.allstop_pvs]
        self.epics_pvs['AllStop'].put(0,wait=True)
        
    def set_all_to_zero(self):
        """Set all user coordinates to zero
        """
        funcs = [self.set_phasering_to_zero,
                 self.set_diffuser_to_zero,
                 self.set_beamstop_to_zero,
                 self.set_pinhole_to_zero,
                 self.set_condenser_to_zero,
                 self.set_zone_plate_to_zero,
                 ]
        threads = [threading.Thread(target=f, args=()) for f in funcs]
        [t.start() for t in threads]
        [t.join() for t in threads]

        self.epics_pvs['SetAllToZero'].put('Done')

    def set_phasering_to_zero(self):
        """Set the phase ring user coordinate to zero.
        """
        if(self.epics_pvs['PhaseRingSetUserCoordinateToZeroUse'].value):
            self.epics_pvs['PhaseRingYSet'].put('Set', wait=True)
            self.epics_pvs['PhaseRingY'].put(0, wait=True)
            self.epics_pvs['PhaseRingYSet'].put('Use', wait=True)
        self.epics_pvs['SetPhaseRingToZero'].put('Done')

    def set_diffuser_to_zero(self):
        """Set the diffuser user coordinate to zero.
        """
        if(self.epics_pvs['DiffuserSetUserCoordinateToZeroUse'].value):
            self.epics_pvs['DiffuserXSet'].put('Set', wait=True)
            self.epics_pvs['DiffuserX'].put(0, wait=True)
            self.epics_pvs['DiffuserXSet'].put('Use', wait=True)
        self.epics_pvs['SetDiffuserToZero'].put('Done')

    def set_beamstop_to_zero(self):
        """Set the beamstop user coordinate to zero.
        """
        if(self.epics_pvs['BeamstopSetUserCoordinateToZeroUse'].value):
            self.epics_pvs['BeamstopYSet'].put('Set', wait=True)
            self.epics_pvs['BeamstopY'].put(0, wait=True)
            self.epics_pvs['BeamstopYSet'].put('Use', wait=True)
        self.epics_pvs['SetBeamstopToZero'].put('Done')

    def set_pinhole_to_zero(self):
        """Set the pinhole user coordinate to zero.
        """
        if(self.epics_pvs['PinholeSetUserCoordinateToZeroUse'].value):
            self.epics_pvs['PinholeYSet'].put('Set', wait=True)
            self.epics_pvs['PinholeY'].put(0, wait=True)
            self.epics_pvs['PinholeYSet'].put('Use', wait=True)
        self.epics_pvs['SetPinholeToZero'].put('Done')

    def set_condenser_to_zero(self):
        """Set the condenser user coordinate to zero.
        """
        if(self.epics_pvs['CondenserSetUserCoordinateToZeroUse'].value):
            self.epics_pvs['CondenserYSet'].put('Set', wait=True)
            self.epics_pvs['CondenserY'].put(0, wait=True)
            self.epics_pvs['CondenserYSet'].put('Use', wait=True)
        self.epics_pvs['SetCondenserToZero'].put('Done')

    def set_zone_plate_to_zero(self):
        """Set the zone plate user coordinate to zero.
        """
        if(self.epics_pvs['ZonePlateSetUserCoordinateToZeroUse'].value):
            self.epics_pvs['ZonePlateYSet'].put('Set', wait=True)
            self.epics_pvs['ZonePlateY'].put(0, wait=True)
            self.epics_pvs['ZonePlateYSet'].put('Use', wait=True)
        self.epics_pvs['SetZonePlateToZero'].put('Done')

    def set_sample_top_to_zero(self):
        """Set the zone plate user coordinate to zero.
        """
        if(self.epics_pvs['SampleTopSetUserCoordinateToZeroUse'].value):
            self.epics_pvs['SampleTopXSet'].put('Set', wait=True)
            self.epics_pvs['SampletopX'].put(0, wait=True)
            self.epics_pvs['SampleTopXSet'].put('Use', wait=True)
        self.epics_pvs['SetSampleTopToZero'].put('Done')
