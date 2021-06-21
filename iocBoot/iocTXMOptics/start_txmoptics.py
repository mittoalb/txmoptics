# This script creates an object of type TXMOptics for doing tomography streaming reconstruction
# To run this script type the following:
#     python -i start_txmoptics.py
# The -i is needed to keep Python running, otherwise it will create the object and exit
from txmoptics.txmoptics import TXMOptics
ts = TXMOptics(["../../db/txmOptics_settings.req","../../db/txmOptics_settings.req"], {"$(P)":"32id:", "$(R)":"TXMOptics:"})
