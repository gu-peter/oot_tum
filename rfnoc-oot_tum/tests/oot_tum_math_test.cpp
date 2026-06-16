//
// Copyright 2026 <author>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <rfnoc/oot_tum/ofdm_tx_sl_block_control.hpp>
#include <boost/test/unit_test.hpp>

using namespace rfnoc::oot_tum;

// Sanity check that the ctrlport register addresses used by
// ofdm_tx_sl_block_control are distinct.
BOOST_AUTO_TEST_CASE(ofdm_tx_sl_register_addresses_test)
{
    BOOST_CHECK_NE(ofdm_tx_sl_block_control::REG_TX_PAYLOAD_READY,
        ofdm_tx_sl_block_control::REG_ENABLE);
}
