import pvaccess as pva
import numpy as np
import time
import threading

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

        # Only create PVs if the prefixes exist
        if 'ValvesPLC' in self.pv_prefixes:
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

        if 'Shaker' in self.pv_prefixes:
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

        if 'BPM' in self.pv_prefixes:
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

        if 'Camera' in self.pv_prefixes:
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

        # Add callbacks only for PVs that exist
        for epics_pv in ('MovePhaseRingIn', 'MovePhaseRingOut', 'MoveDiffuserIn',
                         'MoveDiffuserOut', 'MoveBeamstopIn', 'MoveBeamstopOut', 'MovePinholeIn', 'MovePinholeOut',
                         'MoveCondenserIn', 'MoveCondenserOut', 'MoveZonePlateIn', 'MoveZonePlateOut', 'MoveFurnaceIn', 'MoveFurnaceOut',
                         'MoveSampleIn','MoveSampleOut',
                         'MoveAllIn', 'MoveAllOut', 'AllStop', 'SaveAllPVs', 'LoadAllPVs', 'CrossSelect', 'EnergySet', 'Crop'):
            if epics_pv in self.epics_pvs:
                self.epics_pvs[epics_pv].put(0)
                self.epics_pvs[epics_pv].add_callback(self.pv_callback)
                
        for epics_pv in ('ShutterBClose', 'ShutterBStatus'):
            if epics_pv in self.epics_pvs:
                self.epics_pvs[epics_pv].add_callback(self.pv_callback)
                
        if 'EnergyBusy' in self.epics_pvs:
            self.epics_pvs['EnergyBusy'].put(0,wait=True)
        
        # Start the watchdog timer thread
        thread = threading.Thread(target=self.reset_watchdog, args=(), daemon=True)
        thread.start()
        
        #topx topz pseudo motors
        
        #self.alpha = np.radians(20) #lamino angle
        #self.epics_pvs['PseudoX'].add_callback(self.move_pseudox_callback)
        #self.epics_pvs['PseudoY'].add_callback(self.move_pseudoy_callback)

        #readback_thread = threading.Thread(target=self.update_pseudomotor_readbacks, daemon=True)
        #readback_thread.start()        
        
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

            print(line)
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
            print("pvname", pvname)
            # Do macro substitution on the pvName
            for key in macros:
                pvname = pvname.replace(key, macros[key])
                
            if not pvname or pvname.strip() == "":
                print(f"Skipping empty or invalid PV line: '{line}'")
                continue

            try:
                epics_pv = PV(pvname)
            except Exception as e:
                print(f"Skipping PV '{pvname}' due to error: {e}")
                continue    
            # Replace macros in dictionary key with nothing
            dictentry = line
            for key in macros:
                dictentry = dictentry.replace(key, '')

            if is_config_pv:
                self.config_pvs[dictentry] = epics_pv
            else:
                self.control_pvs[dictentry] = epics_pv
            
            # Handle PVName entries - these contain the actual PV name as their value
            if dictentry.find('PVName') != -1:
                try:
                    # Wait a bit for the PV to connect and get its value
                    time.sleep(0.1)
                    actual_pvname = epics_pv.value
                    if actual_pvname is not None and str(actual_pvname).strip():
                        key = dictentry.replace('PVName', '')
                        print(f"Creating PV for {key}: {actual_pvname}")
                        self.control_pvs[key] = PV(str(actual_pvname).strip())
                    else:
                        print(f"Warning: PV '{pvname}' returned None or empty value, skipping creation of {dictentry.replace('PVName', '')}")
                except Exception as e:
                    print(f"Error creating PV from {dictentry}: {e}")
            
            # Handle PVPrefix entries - these contain PV prefixes as their value        
            if dictentry.find('PVPrefix') != -1:
                try:
                    time.sleep(0.1)
                    pvprefix = epics_pv.value
                    if pvprefix is not None and str(pvprefix).strip():
                        key = dictentry.replace('PVPrefix', '')
                        self.pv_prefixes[key] = str(pvprefix).strip()
                        print(f"Setting prefix for {key}: {pvprefix}")
                    else:
                        print(f"Warning: PV '{pvname}' returned None or empty prefix value for {dictentry}")
                except Exception as e:
                    print(f"Error getting prefix from {dictentry}: {e}")

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
            try:
                print(config_pv, ':', self.config_pvs[config_pv].get(as_string=True))
            except:
                print(config_pv, ': Unable to read')

        print('')
        print('controlPVS:')
        for control_pv in self.control_pvs:
            try:
                print(control_pv, ':', self.control_pvs[control_pv].get(as_string=True))
            except:
                print(control_pv, ': Unable to read')

        print('')
        print('pv_prefixes:')
        for pv_prefix in self.pv_prefixes:
            print(pv_prefix, ':', self.pv_prefixes[pv_prefix])

    def pv_callback(self, pvname=None, value=None, char_value=None, **kw):
        """Callback function that is called by pyEpics when certain EPICS PVs are changed
        """

        log.debug('pv_callback pvName=%s, value=%s, char_value=%s', pvname, value, char_value)
        if (pvname.find('MovePhaseRingIn') != -1) and (value == 1):
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
        elif (pvname.find('MoveSampleIn') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_sample_in, args=())
            thread.start()
        elif (pvname.find('MoveSampleOut') != -1) and (value == 1):
            thread = threading.Thread(target=self.move_sample_out, args=())
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
        elif (pvname.find('Crop') != -1) and (value ==1):
            thread = threading.Thread(target=self.crop_detector, args=())
            thread.start()            
        
    def move_diffuser_in(self):
        """Moves the diffuser in.
        """
        if('DiffuserInOutUse' in self.epics_pvs and self.epics_pvs['DiffuserInOutUse'].value):
            if 'DiffuserInX' in self.epics_pvs and 'DiffuserX' in self.epics_pvs:
                position = self.epics_pvs['DiffuserInX'].value
                self.epics_pvs['DiffuserX'].put(position, wait=True)

        if 'MoveDiffuserIn' in self.epics_pvs:
            self.epics_pvs['MoveDiffuserIn'].put('Done')

    def move_diffuser_out(self):
        """Moves the diffuser out.
        """
        if('DiffuserInOutUse' in self.epics_pvs and self.epics_pvs['DiffuserInOutUse'].value):
            if 'DiffuserOutX' in self.epics_pvs and 'DiffuserX' in self.epics_pvs:
                position = self.epics_pvs['DiffuserOutX'].value
                self.epics_pvs['DiffuserX'].put(position, wait=True)

        if 'MoveDiffuserOut' in self.epics_pvs:
            self.epics_pvs['MoveDiffuserOut'].put('Done')

    def move_beamstop_in(self):
        """Moves the beamstop in.
        """
        if('BeamstopInOutUse' in self.epics_pvs and self.epics_pvs['BeamstopInOutUse'].value):
            if 'BeamstopInY' in self.epics_pvs and 'BeamstopY' in self.epics_pvs:
                position = self.epics_pvs['BeamstopInY'].value
                self.epics_pvs['BeamstopY'].put(position, wait=True)
            if 'BeamstopInX' in self.epics_pvs and 'BeamstopX' in self.epics_pvs:
                position = self.epics_pvs['BeamstopInX'].value
                self.epics_pvs['BeamstopX'].put(position, wait=True) 

        if 'MoveBeamstopIn' in self.epics_pvs:
            self.epics_pvs['MoveBeamstopIn'].put('Done')

    def move_beamstop_out(self):
        """Moves the beamstop out.
        """
        if('BeamstopInOutUse' in self.epics_pvs and self.epics_pvs['BeamstopInOutUse'].value):
            if 'BeamstopOutY' in self.epics_pvs and 'BeamstopY' in self.epics_pvs:
                position = self.epics_pvs['BeamstopOutY'].value
                self.epics_pvs['BeamstopY'].put(position, wait=True)
            if 'BeamstopOutX' in self.epics_pvs and 'BeamstopX' in self.epics_pvs:
                position = self.epics_pvs['BeamstopOutX'].value
                self.epics_pvs['BeamstopX'].put(position, wait=True)

        if 'MoveBeamstopOut' in self.epics_pvs:
            self.epics_pvs['MoveBeamstopOut'].put('Done')

    def move_pinhole_in(self):
        """Moves the pinhole in.
        """
        if('PinholeInOutUse' in self.epics_pvs and self.epics_pvs['PinholeInOutUse'].value):
            if 'PinholeInY' in self.epics_pvs and 'PinholeY' in self.epics_pvs:
                position = self.epics_pvs['PinholeInY'].value
                self.epics_pvs['PinholeY'].put(position, wait=True)
            if 'PinholeInX' in self.epics_pvs and 'PinholeX' in self.epics_pvs:
                position = self.epics_pvs['PinholeInX'].value
                self.epics_pvs['PinholeX'].put(position, wait=True)            

        if 'MovePinholeIn' in self.epics_pvs:
            self.epics_pvs['MovePinholeIn'].put('Done')

    def move_pinhole_out(self):
        """Moves the pinhole out.
        """
        if('PinholeInOutUse' in self.epics_pvs and self.epics_pvs['PinholeInOutUse'].value):
            if 'PinholeOutY' in self.epics_pvs and 'PinholeY' in self.epics_pvs:
                position = self.epics_pvs['PinholeOutY'].value
                self.epics_pvs['PinholeY'].put(position, wait=True)
            if 'PinholeOutX' in self.epics_pvs and 'PinholeX' in self.epics_pvs:
                position = self.epics_pvs['PinholeOutX'].value
                self.epics_pvs['PinholeX'].put(position, wait=True)
            
        if 'MovePinholeOut' in self.epics_pvs:
            self.epics_pvs['MovePinholeOut'].put('Done')

    def move_condenser_in(self):
        """Moves the condenser in.
        """
        if('CondenserInOutUse' in self.epics_pvs and self.epics_pvs['CondenserInOutUse'].value):
            if 'CondenserInY' in self.epics_pvs and 'CondenserY' in self.epics_pvs:
                position = self.epics_pvs['CondenserInY'].value
                self.epics_pvs['CondenserY'].put(position, wait=True)
            if 'CondenserInX' in self.epics_pvs and 'CondenserX' in self.epics_pvs:
                position = self.epics_pvs['CondenserInX'].value
                self.epics_pvs['CondenserX'].put(position, wait=True)
            
        if 'MoveCondenserIn' in self.epics_pvs:
            self.epics_pvs['MoveCondenserIn'].put('Done')

    def move_condenser_out(self):
        """Moves the condenser out.
        """
        if('CondenserInOutUse' in self.epics_pvs and self.epics_pvs['CondenserInOutUse'].value):
            if 'CondenserOutY' in self.epics_pvs and 'CondenserY' in self.epics_pvs:
                position = self.epics_pvs['CondenserOutY'].value
                self.epics_pvs['CondenserY'].put(position, wait=True)
            if 'CondenserOutX' in self.epics_pvs and 'CondenserX' in self.epics_pvs:
                position = self.epics_pvs['CondenserOutX'].value
                self.epics_pvs['CondenserX'].put(position, wait=True)
            
        if 'MoveCondenserOut' in self.epics_pvs:
            self.epics_pvs['MoveCondenserOut'].put('Done')

    def move_zoneplate_in(self):
        """Moves the zone plate in.
        """
        if('ZonePlateInOutUse' in self.epics_pvs and self.epics_pvs['ZonePlateInOutUse'].value):
            if 'ZonePlateInY' in self.epics_pvs and 'ZonePlateY' in self.epics_pvs:
                position = self.epics_pvs['ZonePlateInY'].value
                self.epics_pvs['ZonePlateY'].put(position, wait=True)
            if 'ZonePlateInX' in self.epics_pvs and 'ZonePlateX' in self.epics_pvs:
                position = self.epics_pvs['ZonePlateInX'].value
                self.epics_pvs['ZonePlateX'].put(position, wait=True)
            
        if 'MoveZonePlateIn' in self.epics_pvs:
            self.epics_pvs['MoveZonePlateIn'].put('Done')

    def move_zoneplate_out(self):
        """Moves the zone plate out.
        """
        if('ZonePlateInOutUse' in self.epics_pvs and self.epics_pvs['ZonePlateInOutUse'].value):
            if 'ZonePlateOutY' in self.epics_pvs and 'ZonePlateY' in self.epics_pvs:
                position = self.epics_pvs['ZonePlateOutY'].value
                self.epics_pvs['ZonePlateY'].put(position, wait=True)
            if 'ZonePlateOutX' in self.epics_pvs and 'ZonePlateX' in self.epics_pvs:
                position = self.epics_pvs['ZonePlateOutX'].value
                self.epics_pvs['ZonePlateX'].put(position, wait=True)
            
        if 'MoveZonePlateOut' in self.epics_pvs:
            self.epics_pvs['MoveZonePlateOut'].put('Done')

    def move_phasering_in(self):
        """Moves the phase ring in.
        """
        if('PhaseRingInOutUse' in self.epics_pvs and self.epics_pvs['PhaseRingInOutUse'].value):
            if 'PhaseRingInY' in self.epics_pvs and 'PhaseRingY' in self.epics_pvs:
                position = self.epics_pvs['PhaseRingInY'].value
                self.epics_pvs['PhaseRingY'].put(position, wait=True)
            if 'PhaseRingInX' in self.epics_pvs and 'PhaseRingX' in self.epics_pvs:
                position = self.epics_pvs['PhaseRingInX'].value
                self.epics_pvs['PhaseRingX'].put(position, wait=True)
            
        if 'MovePhaseRingIn' in self.epics_pvs:
            self.epics_pvs['MovePhaseRingIn'].put('Done')

    def move_phasering_out(self):
        """Moves the phase ring out.
        """
        if('PhaseRingInOutUse' in self.epics_pvs and self.epics_pvs['PhaseRingInOutUse'].value):
            if 'PhaseRingOutY' in self.epics_pvs and 'PhaseRingY' in self.epics_pvs:
                position = self.epics_pvs['PhaseRingOutY'].value
                self.epics_pvs['PhaseRingY'].put(position, wait=True)
            if 'PhaseRingOutX' in self.epics_pvs and 'PhaseRingX' in self.epics_pvs:
                position = self.epics_pvs['PhaseRingOutX'].value
                self.epics_pvs['PhaseRingX'].put(position, wait=True)
            
        if 'MovePhaseRingOut' in self.epics_pvs:
            self.epics_pvs['MovePhaseRingOut'].put('Done')

    def move_sample_in(self):
        """Moves the sample in.
        """
        if('SampleInOutUse' in self.epics_pvs and self.epics_pvs['SampleInOutUse'].value):
            if 'SampleInX' in self.epics_pvs and 'SampleX' in self.epics_pvs:
                position = self.epics_pvs['SampleInX'].value
                self.epics_pvs['SampleX'].put(position, wait=True)
            if 'SampleInZ' in self.epics_pvs and 'SampleZ' in self.epics_pvs:
                position = self.epics_pvs['SampleInZ'].value
                self.epics_pvs['SampleZ'].put(position, wait=True)
            
        if 'MoveSampleIn' in self.epics_pvs:
            self.epics_pvs['MoveSampleIn'].put('Done')
            
    def move_sample_out(self):
        """Moves the sample out.
        """
        if('SampleInOutUse' in self.epics_pvs and self.epics_pvs['SampleInOutUse'].value):
            if 'SampleOutX' in self.epics_pvs and 'SampleX' in self.epics_pvs:
                position = self.epics_pvs['SampleOutX'].value
                self.epics_pvs['SampleX'].put(position, wait=True)
            if 'SampleOutZ' in self.epics_pvs and 'SampleZ' in self.epics_pvs:
                position = self.epics_pvs['SampleOutZ'].value
                self.epics_pvs['SampleZ'].put(position, wait=True)
            
        if 'MoveSampleOut' in self.epics_pvs:
            self.epics_pvs['MoveSampleOut'].put('Done')            


    def move_furnace_in(self):
        """Moves the furnace in.
        """
        if('FurnaceInOutUse' in self.epics_pvs and self.epics_pvs['FurnaceInOutUse'].value):
            if 'FurnaceInY' in self.epics_pvs and 'FurnaceY' in self.epics_pvs:
                position = self.epics_pvs['FurnaceInY'].value
                self.epics_pvs['FurnaceY'].put(position, wait=True)
            if 'FurnaceInX' in self.epics_pvs and 'FurnaceX' in self.epics_pvs:
                position = self.epics_pvs['FurnaceInX'].value
                self.epics_pvs['FurnaceX'].put(position, wait=True)
            
        if 'MoveFurnaceIn' in self.epics_pvs:
            self.epics_pvs['MoveFurnaceIn'].put('Done')

    def move_furnace_out(self):
        """Moves the furnace out.
        """
        if('FurnaceInOutUse' in self.epics_pvs and self.epics_pvs['FurnaceInOutUse'].value):
            if 'FurnaceOutY' in self.epics_pvs and 'FurnaceY' in self.epics_pvs:
                position = self.epics_pvs['FurnaceOutY'].value
                self.epics_pvs['FurnaceY'].put(position, wait=True)
            if 'FurnaceOutX' in self.epics_pvs and 'FurnaceX' in self.epics_pvs:
                position = self.epics_pvs['FurnaceOutX'].value
                self.epics_pvs['FurnaceX'].put(position, wait=True)
            
        if 'MoveFurnaceOut' in self.epics_pvs:
            self.epics_pvs['MoveFurnaceOut'].put('Done')

    def set_exposure_time_in(self):
        """Set exposure time in.
        """
        if('ExposureTimeInOutUse' in self.epics_pvs and self.epics_pvs['ExposureTimeInOutUse'].value):
            if 'ExposureTimeIn' in self.epics_pvs:
                exposure_time = self.epics_pvs['ExposureTimeIn'].value
                if 'CamAcquireTime' in self.control_pvs:
                    self.control_pvs['CamAcquireTime'].put(exposure_time, wait=True, timeout=10.0)

    def set_exposure_time_out(self):
        """Set exposure time out.
        """
        if('ExposureTimeInOutUse' in self.epics_pvs and self.epics_pvs['ExposureTimeInOutUse'].value):
            if 'ExposureTimeOut' in self.epics_pvs:
                exposure_time = self.epics_pvs['ExposureTimeOut'].value
                if 'CamAcquireTime' in self.control_pvs:
                    self.control_pvs['CamAcquireTime'].put(exposure_time, wait=True, timeout=10.0)

    def set_bpm_in(self):
        """
        Set BPM readback value in
        """
        if('BPMSetPointInOutUse' in self.epics_pvs and self.epics_pvs['BPMSetPointInOutUse'].value):
            if 'BPMVSetPointIn' in self.epics_pvs and 'BPMVSetPoint' in self.control_pvs:
                positionv = self.epics_pvs['BPMVSetPointIn'].value
                print(self.control_pvs['BPMVSetPoint'].get(), positionv)
                self.control_pvs['BPMVSetPoint'].put(positionv, wait=True)
            if 'BPMHSetPointIn' in self.epics_pvs and 'BPMHSetPoint' in self.control_pvs:
                positionh = self.epics_pvs['BPMHSetPointIn'].value
                self.control_pvs['BPMHSetPoint'].put(positionh, wait=True)            
    
    def set_bpm_out(self):
        """
        Set BPM readback value out
        """
        if('BPMSetPointInOutUse' in self.epics_pvs and self.epics_pvs['BPMSetPointInOutUse'].value):
            if 'BPMVSetPointOut' in self.epics_pvs and 'BPMVSetPoint' in self.control_pvs:
                positionv = self.epics_pvs['BPMVSetPointOut'].value
                self.control_pvs['BPMVSetPoint'].put(positionv, wait=True)
            if 'BPMHSetPointOut' in self.epics_pvs and 'BPMHSetPoint' in self.control_pvs:
                positionh = self.epics_pvs['BPMHSetPointOut'].value
                self.control_pvs['BPMHSetPoint'].put(positionh, wait=True)            

    def transform_image_in(self):
        """
        Transform image in 
        """
        if 'CamTrans1Type' in self.control_pvs:
            self.control_pvs['CamTrans1Type'].put(2, wait=True) # None
        
    def transform_image_out(self):
        """
        Transform image out 
        """
        if 'CamTrans1Type' in self.control_pvs:
            self.control_pvs['CamTrans1Type'].put(0, wait=True) # Rotate180
                
    def move_all_in(self):
        """Moves all in
        """
        funcs = [self.set_bpm_in,
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

        if 'MoveAllIn' in self.epics_pvs:
            self.epics_pvs['MoveAllIn'].put('Done')

    def move_all_out(self):
        """Moves all out
        """
        funcs = [self.set_bpm_out,
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

        if 'MoveAllOut' in self.epics_pvs:
            self.epics_pvs['MoveAllOut'].put('Done')
        
    def all_stop(self):
        """Stop all iocs motors
        """     
        iocs = [self.pv_prefixes['IOC'+str(k)] for k in range(5) if 'IOC'+str(k) in self.pv_prefixes]
        print(iocs)
        allstop_pvs = [PV(ioc+'allstop') for ioc in iocs]                        
        [pv.put(1,wait=True) for pv in allstop_pvs]
        if 'AllStop' in self.epics_pvs:
            self.epics_pvs['AllStop'].put(0,wait=True)
    
    def save_all_pvs(self):
        """Save all PVs from txm_main.adl screen to a file
        """
        if('LoadAllPVs' in self.epics_pvs and self.epics_pvs['LoadAllPVs'].get()==1):
            if 'SaveAllPVs' in self.epics_pvs:
                self.epics_pvs['SaveAllPVs'].put(0,wait=True)       
            return
        
        if 'FileAllPVs' not in self.epics_pvs:
            print("FileAllPVs PV not found")
            return
            
        file_name = self.epics_pvs['FileAllPVs'].get()
        
        # read adl file
        try:
            with open('/home/beams/USERTXM/epics/synApps/support/txmoptics/txmOpticsApp/op/adl/txm_main.adl','r') as fid:    
                s = fid.read()
        except FileNotFoundError:
            print("ADL file not found, skipping PV extraction")
            if 'SaveAllPVs' in self.epics_pvs:
                self.epics_pvs['SaveAllPVs'].put(0,wait=True)
            return
            
        # take pvs
        pvs = []
        pvs = re.findall(r"chan=\"(.*?)\"", s)
        
        print(pvs)
        # save values to a txt file 
        try:
            with open(file_name,'w') as fid:
                if 'EnergyMonochromator' in self.control_pvs:
                    energy = self.control_pvs['EnergyMonochromator'].get()
                    fid.write('energy '+ str(energy) +'\n')                
                for k in pvs:
                    if (k.find('.VAL')!=-1 and 
                        k.find('32idcTXM:mcs:c0')==-1 and 
                        k.find('32idcSoft:nf:c0')==-1 and 
                        k.find('32idaSoft:m10')==-1 and 
                        k.find('32idcSOFT:nf:c0:m3')==-1 and 
                        k.find('32idaSoft:m9')==-1 and 
                        k.find('32idcTXM:mxv:c1:')==-1): ## temporarily avoid pvs
                        try:
                            print(k)
                            p = PV(k)
                            time.sleep(0.1)
                            val = p.get(as_string=True,timeout=30)
                            if(val is not None and isfloat(val)):
                                print(k,val)                        
                                fid.write(k[:-4]+' '+val+'\n')
                        except:
                            pass
        except:
            log.error('File %s cannot be created', file_name)
        
        if 'SaveAllPVs' in self.epics_pvs:
            self.epics_pvs['SaveAllPVs'].put(0,wait=True)        
    
    def load_all_pvs(self):
        """Load all PVs to txm_main.adl screen to a file
        """
        if('SaveAllPVs' in self.epics_pvs and self.epics_pvs['SaveAllPVs'].get()==1):
            if 'LoadAllPVs' in self.epics_pvs:
                self.epics_pvs['LoadAllPVs'].put(0,wait=True)        
            return
        
        if 'FileAllPVs' not in self.epics_pvs:
            print("FileAllPVs PV not found")
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
        
        if 'LoadAllPVs' in self.epics_pvs:
            self.epics_pvs['LoadAllPVs'].put(0,wait=True)

    def cross_select(self):
        """Plot the cross in imageJ.
        """
        if 'CrossSelect' not in self.epics_pvs:
            return
            
        if (self.epics_pvs['CrossSelect'].get() == 0):
            if ('CamSizeXRBV' in self.epics_pvs and 'CamSizeYRBV' in self.epics_pvs and
                'OP1CenterX' in self.epics_pvs and 'OP1CenterY' in self.epics_pvs and
                'OP2CenterX' in self.epics_pvs and 'OP2CenterY' in self.epics_pvs and
                'OP1Use' in self.control_pvs and 'OP2Use' in self.control_pvs):
                
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
            if 'OP1Use' in self.control_pvs and 'OP2Use' in self.control_pvs:
                self.control_pvs['OP1Use'].put(0)
                self.control_pvs['OP2Use'].put(0)
                log.info('Cross is disabled')

    def shutter_b_close(self):        
        if ('ShutterCallback' in self.epics_pvs and self.epics_pvs['ShutterCallback'].get() == 0):
            log.info('Stop BPM')
            if 'BPMVFeedback' in self.control_pvs:
                self.control_pvs['BPMVFeedback'].put(0)
            if 'BPMHFeedback' in self.control_pvs:
                self.control_pvs['BPMHFeedback'].put(0)  
        
    def shutter_b_status(self):
        if ('ShutterCallback' in self.epics_pvs and self.epics_pvs['ShutterCallback'].get() == 0):
            log.info('Start BPM')
            if 'BPMVFeedback' in self.control_pvs:
                self.control_pvs['BPMVFeedback'].put(1)
            if 'BPMHFeedback' in self.control_pvs:
                self.control_pvs['BPMHFeedback'].put(1)  

    def energy_change(self):
        
        if ('EnergyBusy' not in self.epics_pvs or self.epics_pvs['EnergyBusy'].get() == 0):
            if 'EnergyBusy' in self.epics_pvs:
                self.epics_pvs['EnergyBusy'].put(1)
            # if 'BPMVFeedback' in self.control_pvs:
            #     self.control_pvs['BPMVFeedback'].put(0)
            # if 'BPMHFeedback' in self.control_pvs:
            #     self.control_pvs['BPMHFeedback'].put(0)      
            
            if 'Energy' and 'EnergyDetune' in self.epics_pvs:
                energy = float(self.epics_pvs["Energy"].get())
                energyDetune = float(self.epics_pvs["EnergyDetune"].get())
                log.info("TxmOptics: change energy to %.2f",energy)
            else:
                log.error("Energy PV not found")
                return
            
            log.info('move monochromator')
            if 'DCMputEnergy' in self.epics_pvs:
                self.epics_pvs['DCMputEnergy'].put(energy)
            log.info('move undulator')
            if 'GAPputEnergy' in self.epics_pvs:
                print(self.epics_pvs['GAPputEnergy'])
                self.epics_pvs['GAPputEnergy'].put(energy+energyDetune/1000)
                print('GAPputEnergy done')
                time.sleep(0.2)
                self.epics_pvs['GAPputEnergyStart'].put(1)
                print('GAPputEnergyStart done')
            time.sleep(0.2)# possible backlash/stabilization, more??
            log.info('skip wait mono')
            log.info('skip wait undulator')
            print(self.epics_pvs['EnergyCalibrationFileOne'].get())
            print(self.epics_pvs['EnergyCalibrationFileTwo'].get())
            
            # time.sleep(1)# possible backlash/stabilization, more??
            if ('EnergyUseCalibration' in self.epics_pvs and 
                self.epics_pvs['EnergyUseCalibration'].get(as_string=True) == 'Yes'):                
                try:
                    # read pvs for 2 energies
                    pvs1, pvs2, vals1, vals2 = [],[],[],[]
                    
                    if ('EnergyCalibrationFileOne' in self.epics_pvs and 
                        'EnergyCalibrationFileTwo' in self.epics_pvs):
                        
                        with open(self.epics_pvs['EnergyCalibrationFileOne'].get()) as fid:
                            for pv_val in fid.readlines():
                                print(pv_val)
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
                                raise Exception("PV mismatch in calibration files")                            
                        if(np.abs(vals2[0]-vals1[0])<0.001):            
                            raise Exception("Energy values too close in calibration files")           
                        vals = []                     
                        for k in range(len(pvs1)):
                            vals.append(vals1[k]+(energy-vals1[0])*(vals2[k]-vals1[k])/(vals2[0]-vals1[0]))               
                        # set new pvs  
                        for k in range(1,len(pvs1)):# skip energy line                        
                            if ('DetectorZ' in self.epics_pvs and 
                                pvs1[k]==self.epics_pvs['DetectorZ'].pvname):                            
                                log.info('old Detector Z %3.3f', self.epics_pvs['DetectorZ'].get())
                                self.epics_pvs['DetectorZ'].put(vals[k],wait=True)                                                        
                                log.info('new Detector Z %3.3f', self.epics_pvs['DetectorZ'].get())
                            if ('ZonePlateZ' in self.epics_pvs and 
                                pvs1[k]==self.epics_pvs['ZonePlateZ'].pvname):                            
                                log.info('old Zone plate Z %3.3f', self.epics_pvs['ZonePlateZ'].get())
                                self.epics_pvs['ZonePlateZ'].put(vals[k],wait=True)                                                        
                                log.info('new Zone plate Z %3.3f', self.epics_pvs['ZonePlateZ'].get())
                except Exception as e:
                    log.error('Calibration files error: %s', str(e))
                    
            # if 'BPMVFeedback' in self.control_pvs:
            #     self.control_pvs['BPMVFeedback'].put(1)
            # if 'BPMHFeedback' in self.control_pvs:
            #     self.control_pvs['BPMHFeedback'].put(1)                                  
            log.info('energy change is done')
            if 'EnergyBusy' in self.epics_pvs:
                self.epics_pvs['EnergyBusy'].put(0)   
            if 'EnergySet' in self.epics_pvs:
                self.epics_pvs['EnergySet'].put(0)      

    def crop_detector(self):
        """crop detector sizes"""
        if 'CamAcquire' not in self.control_pvs:
            return
            
        state = self.control_pvs['CamAcquire'].get()
        self.control_pvs['CamAcquire'].put(0,wait=True)

        if ('CamMaxSizeXRBV' in self.epics_pvs and 'CamMaxSizeYRBV' in self.epics_pvs and
            'CamMinX' in self.control_pvs and 'CamMinY' in self.control_pvs and
            'CamSizeX' in self.control_pvs and 'CamSizeY' in self.control_pvs):
            
            maxsizex = self.epics_pvs['CamMaxSizeXRBV'].get()
            self.control_pvs['CamMinX'].put(0,wait=True)        
            
            maxsizey = self.epics_pvs['CamMaxSizeYRBV'].get()
            self.control_pvs['CamMinY'].put(0,wait=True)        
            
            if ('CropLeft' in self.epics_pvs and 'CropBottom' in self.epics_pvs and
                'CropRight' in self.epics_pvs and 'CropTop' in self.epics_pvs):
                
                left = self.epics_pvs['CropLeft'].get()
                top = self.epics_pvs['CropBottom'].get()#flipped due to the ZP
                
                right = self.epics_pvs['CropRight'].get()        
                self.control_pvs['CamSizeX'].put(maxsizex-left-right,wait=True)
                
                if 'CamSizeXRBV' in self.epics_pvs:
                    sizex = self.epics_pvs['CamSizeXRBV'].get()
                    right = maxsizex - left - sizex
                    self.epics_pvs['CropRight'].put(right,wait=True)

                bottom = self.epics_pvs['CropTop'].get()
                self.control_pvs['CamSizeY'].put(maxsizey-top-bottom,wait=True)
                
                if 'CamSizeYRBV' in self.epics_pvs:
                    sizey = self.epics_pvs['CamSizeYRBV'].get()
                    bottom = maxsizey - top - sizey
                    self.epics_pvs['CropTop'].put(bottom,wait=True)

                self.control_pvs['CamMinX'].put(left,wait=True)        
                if 'CamMinXRBV' in self.epics_pvs:
                    left = self.epics_pvs['CamMinXRBV'].get()
                    self.epics_pvs['CropLeft'].put(left,wait=True)

                self.control_pvs['CamMinY'].put(top,wait=True)        
                if 'CamMinYRBV' in self.epics_pvs:
                    top = self.epics_pvs['CamMinYRBV'].get()
                    self.epics_pvs['CropBottom'].put(top,wait=True)
                    
        self.control_pvs['CamAcquire'].put(1)  # need to take at least 1 frame
        
        self.cross_select()
        if 'Crop' in self.epics_pvs:
            self.epics_pvs['Crop'].put(0,wait=True)  

    def reset_watchdog(self):
        """Sets the watchdog timer to 5 every 3 seconds"""
        while True:
            if 'Watchdog' in self.epics_pvs:
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
                
                            
    def move_pseudox_callback(self, pvname=None, value=None, **kwargs):
        pseudox = value
        pseudoy = self.epics_pvs['PseudoYReadback'].get()
        theta = np.radians(self.epics_pvs['Rotary'].get())

        alpha = self.alpha
        cos_t, sin_t, cos_a = cos(theta), sin(theta), cos(alpha)

        x1 = pseudox * cos_t - pseudoy * sin_t
        x2 = (pseudox * sin_t + pseudoy * cos_t) * cos_a

        self.epics_pvs['TopX'].put(x1, wait=True)
        self.epics_pvs['TopZ'].put(x2, wait=True)

    def move_pseudoy_callback(self, pvname=None, value=None, **kwargs):
        pseudoy = value
        pseudox = self.epics_pvs['PseudoXReadback'].get()
        theta = np.radians(self.epics_pvs['Rotary'].get())

        alpha = self.alpha
        cos_t, sin_t, cos_a = cos(theta), sin(theta), cos(alpha)

        x1 = pseudox * cos_t - pseudoy * sin_t
        x2 = (pseudox * sin_t + pseudoy * cos_t) * cos_a

        self.epics_pvs['TopX'].put(x1, wait=True)
        self.epics_pvs['TopZ'].put(x2, wait=True)
        
    def update_pseudomotor_readbacks(self):
        while True:
            try:
                x1 = self.epics_pvs['TopX'].get()
                x2 = self.epics_pvs['TopZ'].get()
                theta = np.radians(self.epics_pvs['Rotary'].get())
                alpha = self.alpha

                cos_t, sin_t, cos_a = cos(theta), sin(theta), cos(alpha)

                pseudox = x1 * cos_t + (x2 / cos_a) * sin_t
                pseudoy = -x1 * sin_t + (x2 / cos_a) * cos_t

                self.epics_pvs['PseudoXReadback'].put(pseudox)
                self.epics_pvs['PseudoYReadback'].put(pseudoy)
            except Exception as e:
                log.error(f"Failed to update pseudo readbacks: {e}")
            time.sleep(0.5)
        
        
        
                
