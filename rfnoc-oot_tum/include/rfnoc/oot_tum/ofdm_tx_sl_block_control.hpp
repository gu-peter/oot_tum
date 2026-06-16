//
// Copyright 2026 <author>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <rfnoc/oot_tum/config.hpp>
#include <uhd/rfnoc/noc_block_base.hpp>
#include <cstdint>

namespace rfnoc { namespace oot_tum {

/*! Block controller: Describe me!
 */
class RFNOC_OOT_TUM_API ofdm_tx_sl_block_control : public uhd::rfnoc::noc_block_base
{
public:
    RFNOC_DECLARE_BLOCK(ofdm_tx_sl_block_control)

    // List all registers here if you need to know their address in the block controller:
    //! The register address of the txPayloadReady status flag
    static const uint32_t REG_TX_PAYLOAD_READY;
    //! The register address of the enable control flag
    static const uint32_t REG_ENABLE;

    /*! Returns whether the txPayload input is ready to accept more data.
     *
     * This tells the host when new input samples may be flushed into the
     * block.
     */
    virtual bool get_tx_payload_ready() = 0;

    /*! Enable or disable the OFDM transmitter
     */
    virtual void set_enable(const bool enable) = 0;

    /*! Returns whether the OFDM transmitter is currently enabled
     */
    virtual bool get_enable() = 0;
};

}} // namespace rfnoc::oot_tum
