*******************************
tomoStreamApp EPICS application
*******************************

.. 
   toctree::
   :hidden:

   tomoStream.template
   tomoStream_settings.req
   tomoStream.substitutions


tomostream includes a complete example EPICS application, including:

- A database file and corresponding autosave request file that contain only the PVs required by the tomoscan.py base class.
- Database files and corresponding autosave request files that contain PVs used by the derived classes.
- An example IOC application that can be used to run the above databases.
  The databases are loaded in the IOC with the example substitutions file, 
  :doc:`tomoStream.substitutions`.

Base class files
================
The following tables list all of the records in the tomoScan.template file.
These records are used by the tomoscan base class and so are required.

tomoStream.template
-------------------

This is the database file that contains only the PVs required by the tomoscan.py base class
:doc:`tomoStream.template`.

tomoStream PV Prefixes
~~~~~~~~~~~~~~~~~~~~~~

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
  :header-rows: 1
  :widths: 5 5 90

  * - Record name
    - Record type
    - Description
  * - $(P)$(R)TomoScanPVPrefix
    - stringout
    - Contains the prefix for the tomoscan controlling the data collection, e.g. 2bma:TomoScan

tomoStream PVA Names
~~~~~~~~~~~~~~~~~~~~

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
  :header-rows: 1
  :widths: 5 5 90

  * - Record name
    - Record type
    - Description
  * - $(P)$(R)ImagePVAPName
    - stringout
    - Contains the name of the TomoScan PV storing the PV prefix of the images streamed by the detector
  * - $(P)$(R)DarkPVAName
    - stringout
    - Contains the name of the TomoScan PVA where the dark images are stored
  * - $(P)$(R)FlatPVAName
    - stringout
    - Contains the name of the TomoScan PVA where the flat images are stored
  * - $(P)$(R)ThetaPVAName
    - stringout
    - Contains the name of the TomoScan PVA where the rotation angle positions are stored
  * - $(P)$(R)ReconPVAName
    - stringout
    - Contains the name of the TomoStream PVA where the the selected 3 orthogonal slices are stored

Streaming analysis control
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
  :header-rows: 1
  :widths: 5 5 90

  * - Record name
    - Record type
    - Description
  * - $(P)$(R)CameraPVPrefix
    - stringout
    - Contains the prefix for the camera, e.g. 13BMDPG1:
  * - $(P)$(R)Status
    - bo
    - Flag storing the  streaming status. Choices are 'Off' and 'On'. When 'On' the streaming reconstruction is enabled 
  * - $(P)$(R)BufferSize
    - longout
    - Stream buffer size
  * - $(P)$(R)Center
    - ao
    - Rotation center for streaming reconstruction
  * - $(P)$(R)FilterType
    - mbbo
    - Filter type for streaming reconstruction, 'Parzen', 'Shepp-logan', 'Ramp', 'Butterworth'
  * - $(P)$(R)OrthoX
    - longout
    - Ortho slice in the X direction for streaming reconstruction
  * - $(P)$(R)OrthoY
    - longout
    - Ortho slice in the Y direction for streaming reconstruction
  * - $(P)$(R)OrthoZ
    - longout
    - Ortho slice in the Z direction for streaming reconstruction

Stream status via Channel Access
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
  :header-rows: 1
  :widths: 5 5 90

  * - Record name
    - Record type
    - Description
  * - $(P)$(R)ReconStatus
    - waveform
    - This record will be updated with the stream reconstruction status while scanning.
  * - $(P)$(R)ReconTime
    - ao
    - This record will update with the time to reconstruct the selected 3 orthogonal slices.
  * - $(P)$(R)ServerRunning
    - bi
    - This record will be ``Running`` if the Python server is running and ``Stopped`` if not.
      It is controlled by a watchdog timer, and will change from ``Running`` to ``Stopped``
      within 5 seconds if the Python server exits.


tomoStream_settings.req
~~~~~~~~~~~~~~~~~~~~~~~

This is the autosave request file for tomoStream.template
:doc:`tomoStream_settings.req`.

It has the same usage and type of content as tomoStream_settings.req described above, except that it contains the PVs for the derived class TomoStream.

medm files
~~~~~~~~~~

To start the tomostream medm screen::

  $ cd /local/USERNAME/epics/synApps/support/tomostream/iocBoot/iocTomoStream
  $ start_medm

where USERNAME is the username under which the tomoStreamApp is installed.

tomoStream.adl
^^^^^^^^^^^^^^

The following is the MEDM screen :download:`tomoStream.adl <../../tomoStreamApp/op/adl/tomoStream.adl>`.  
This screen contains the PVs to control tomoStream.

.. image:: img/tomoStream.png
    :width: 75%
    :align: center

tomoStreamEPICS_PVs.adl
^^^^^^^^^^^^^^^^^^^^^^^

The EPICS PV names screen is below:

.. image:: img/tomoStreamEPICS_PVs.png
    :width: 60%
    :align: center



TXM support IOC PV Prefixes
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
  :header-rows: 1
  :widths: 5 5 90

  * - Record name
    - Record type
    - Description
  * - $(P)$(R)CRLRelaysPVPrefix
    - stringout
    - Contains the prefix for the CRL Relays IOC, e.g. 32idbPLC:
  * - $(P)$(R)ValvesPLCPVPrefix
    - stringout
    - Contains the prefix for the Valves PLC IOC, e.g. 32idcPLC:
  * - $(P)$(R)ShakerPVPrefix
    - stringout
    - Contains the prefix for the Shaker IOC, e.g. 32idcMC:shaker:
  * - $(P)$(R)BPMPVPrefix
    - stringout
    - Contains the prefix for the BPM IOC, e.g. 32ida:


TXM Optics motors
^^^^^^^^^^^^^^^^^

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
  :header-rows: 1
  :widths: 5 5 90

  * - Record name
    - Record type
    - Description
  * - $(P)$(R)CRLXPVName
    - stringout
    - Contains the name of the Compound Refractive Lens (CRL) X translation PV, e.g. 32idb:m32
  * - $(P)$(R)CRLYPVName
    - stringout
    - Contains the name of the Compound Refractive Lens (CRL) Y translation PV, e.g. 32idb:m28
  * - $(P)$(R)CRLPitchPVName
    - stringout
    - Contains the name of the Compound Refractive Lens (CRL) Pitch adjustment PV, e.g. 32idb:m26
  * - $(P)$(R)CRLYawPVName
    - stringout
    - Contains the name of the Compound Refractive Lens (CRL) Yaw adjustment PV, e.g. 32idb:m27
  * - $(P)$(R)DiffuserXPVName
    - stringout
    - Contains the name of the Beamstop X translation PV, e.g. 32idcTXM:xps:c1:m2
  * - $(P)$(R)BeamstopXPVName
    - stringout
    - Contains the name of the Beamstop X translation PV, e.g. 32idcTXM:mcs:c3:m3
  * - $(P)$(R)BeamstopYPVName
    - stringout
    - Contains the name of the Beamstop Y translation PV, e.g. 32idcTXM:mcs:c3:m6
  * - $(P)$(R)PinholeXPVName
    - stringout
    - Contains the name of the Pinhole X translation PV, e.g. 32idcTXM:xps:c1:m3
  * - $(P)$(R)PinholeYPVName
    - stringout
    - Contains the name of the Pinhole Y translation PV, e.g. 32idcTXM:xps:c1:m5
  * - $(P)$(R)CondenserXPVName
    - stringout
    - Contains the name of the Condenser X translation PV, e.g. 32idcTXM:mcs:c3:m1
  * - $(P)$(R)CondenserYPVName
    - stringout
    - Contains the name of the Condenser Y translation PV, e.g. 32idcTXM:mcs:c3:m5
  * - $(P)$(R)CondenserZPVName
    - stringout
    - Contains the name of the Condenser Z translation PV, e.g. 32idcTXM:mcs:c1:m5
  * - $(P)$(R)CondenserPitchPVName
    - stringout
    - Contains the name of the Condenser Pitch adjustment PV, e.g. 32idcTXM:mcs:c3:m4
  * - $(P)$(R)CondenserYawPVName
    - stringout
    - Contains the name of the Condenser Yaw adjustment PV, e.g. 32idcTXM:mcs:c3:m2
  * - $(P)$(R)ZonePlateXPVName
    - stringout
    - Contains the name of the Zone plate X translation PV, e.g. 32idcTXM:mcs:c2:m1
  * - $(P)$(R)ZonePlateYPVName
    - stringout
    - Contains the name of the Zone plate Y translation PV, e.g. 32idcTXM:mcs:c2:m2
  * - $(P)$(R)ZonePlateZPVName
    - stringout
    - Contains the name of the Zone plate Z translation PV, e.g. 32idcTXM:mcs:c2:m3
  * - $(P)$(R)PhaseRingXPVName
    - stringout
    - Contains the name of the Phase ring X translation PV, e.g. 32idcSOFT:mmc:c1:m2
  * - $(P)$(R)PhaseRingYPVName
    - stringout
    - Contains the name of the Phase ring Y translation PV, e.g. 32idcSOFT:mmc:c1:m1

Optics control via Channel Access
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
  :header-rows: 1
  :widths: 5 5 90

  * - Record name
    - Record type
    - Description
  * - $(P)$(R)MoveAllIn
    - ao
    - Setting this record to 1 moves all TXM optics in.
  * - $(P)$(R)MoveAllOut
    - ao
    - Setting this record to 1 moves all TXM optics out.
  * - $(P)$(R)MoveCRLIn
    - ao
    - Setting this record to 1 moves CRL in.
  * - $(P)$(R)MoveCRLOut
    - ao
    - Setting this record to 1 moves moves CRL out.
  * - $(P)$(R)MoveDiffuserIn
    - ao
    - Setting this record to 1 moves diffuser in.
  * - $(P)$(R)MoveDiffuserOut
    - ao
    - Setting this record to 1 moves diffuser out.
  * - $(P)$(R)MoveBeamstopIn
    - ao
    - Setting this record to 1 moves beamstop in.
  * - $(P)$(R)MoveBeamstopOut
    - ao
    - Setting this record to 1 moves beamstop out.
  * - $(P)$(R)MovePinholeIn
    - ao
    - Setting this record to 1 moves pinhole in.
  * - $(P)$(R)MovePinholeOut
    - ao
    - Setting this record to 1 moves pinhole out.
  * - $(P)$(R)MoveCondenserIn
    - ao
    - Setting this record to 1 moves condenser in.
  * - $(P)$(R)MoveCondenserOut
    - ao
    - Setting this record to 1 moves condenser out.
  * - $(P)$(R)MoveZonePlateIn
    - ao
    - Setting this record to 1 moves zone plate in.
  * - $(P)$(R)MoveZonePlateOut
    - ao
    - Setting this record to 1 moves zone plate out.
  * - $(P)$(R)MovePhaseRingIn
    - ao
    - Setting this record to 1 moves phase ring in.
  * - $(P)$(R)MovePhaseRingOut
    - ao
    - Setting this record to 1 moves phase ring out.

Optics control
^^^^^^^^^^^^^^

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
  :header-rows: 1
  :widths: 5 5 90

  * - Record name
    - Record type
    - Description
  * - $(P)$(R)NumFlatFields
    - longout
    - Number of flat fields to collect
  * - $(P)$(R)SampleInX
    - ao
    - Position of the X stage when the sample is in position for collecting projections.
  * - $(P)$(R)SampleOutX
    - ao
    - Position of the X stage when the sample is out for collecting flat fields.
  * - $(P)$(R)DiffuserInX
    - ao
    - Position of the X stage when the diffuser is in the beam.
  * - $(P)$(R)DiffuserOutX
    - ao
    - Position of the X stage when the diffuser is out of the beam.
  * - $(P)$(R)BeamstopInY
    - ao
    - Position of the Y stage when the beamstop is in the beam.
  * - $(P)$(R)BeamstopOutY
    - ao
    - Position of the Y stage when the beamstop is out of the beam.
  * - $(P)$(R)PinholeInY
    - ao
    - Position of the Y stage when the pinhole is in the beam.
  * - $(P)$(R)PinholeOutY
    - ao
    - Position of the Y stage when the pinhole is out of the beam.
  * - $(P)$(R)CondenserInY")
    - ao
    - Position of the Y stage when the condenser is in the beam.
  * - $(P)$(R)CondenserOutY
    - ao
    - Position of the Y stage when the condenser is out of the beam.
  * - $(P)$(R)ZonePlateInY
    - ao
    - Position of the Y stage when the zone plate is in the beam.
  * - $(P)$(R)ZonePlateOutY
    - ao
    - Position of the Y stage when the zone plate is out of the beam.
  * - $(P)$(R)PhaseRingInX
    - ao
    - Position of the X stage when the phase ring is in the beam.
  * - $(P)$(R)PhaseRingOutX
    - ao
    - Position of the X stage when the phase ring is out of the beam.
  * - $(P)$(R)PhaseRingInY
    - ao
    - Position of the Y stage when the phase ring is in the beam.
  * - $(P)$(R)PhaseRingOutY
    - ao
    - Position of the Y stage when the phase ring is out of the beam.
  * - $(P)$(R)AllStop
    - bo
    - Stops all TXM optic motors. Options are release or stop.
