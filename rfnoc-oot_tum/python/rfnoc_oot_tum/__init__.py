#
# Copyright 2024 Ettus Research, a National Instruments Company
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""
rfnoc-oot_tum: Example module for Python support of an RFNoC OOT Module
"""

# Import all bindings from C++
from . import rfnoc_oot_tum_python as lib

# In UHD, we use CamelCase for names in Python, so we'll do the same here
OfdmTxSlBlockControl = lib.ofdm_tx_sl_block_control