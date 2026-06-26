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

/*! Block controller for the ofdm_rx_sl block.
 *
 * The OFDM receiver block recovers QPSK payload dibits from a captured OFDM
 * waveform. It exposes the core's two control inputs ('start' and
 * 'freqCorrectionEn') and a set of read-only peakInfo debug registers.
 */
class RFNOC_OOT_TUM_API ofdm_rx_sl_block_control : public uhd::rfnoc::noc_block_base
{
public:
    RFNOC_DECLARE_BLOCK(ofdm_rx_sl_block_control)

    // Register addresses (see rfnoc_block_ofdm_rx_sl.sv).
    //! 'start' control flag (arm synchronization)
    static const uint32_t REG_START;
    //! 'freqCorrectionEn' control flag
    static const uint32_t REG_FREQ_CORR_EN;
    //! Latched peakInfo correlation (ufix32_En24), read-only
    static const uint32_t REG_PEAK_CORRELATION;
    //! Latched peakInfo threshold (ufix32_En24), read-only
    static const uint32_t REG_PEAK_THRESHOLD;
    //! Latched peakInfo timing offset (ufix14), read-only
    static const uint32_t REG_PEAK_TOFFSET;
    //! Latched peakInfo frequency offset (int32), read-only
    static const uint32_t REG_PEAK_FOFFSET;
    //! Number of peakInfoValid strobes seen (debug), read-only
    static const uint32_t REG_PEAK_COUNT;

    /*! Arm/disarm the receiver synchronization ('start' input). */
    virtual void set_start(const bool start) = 0;

    /*! Returns the current 'start' value. */
    virtual bool get_start() = 0;

    /*! Enable/disable frequency correction ('freqCorrectionEn' input). */
    virtual void set_freq_correction_en(const bool enable) = 0;

    /*! Returns the current 'freqCorrectionEn' value. */
    virtual bool get_freq_correction_en() = 0;

    /*! Returns the latched peakInfo correlation (raw ufix32_En24). */
    virtual uint32_t get_peak_correlation() = 0;

    /*! Returns the latched peakInfo threshold (raw ufix32_En24). */
    virtual uint32_t get_peak_threshold() = 0;

    /*! Returns the latched peakInfo timing offset (ufix14). */
    virtual uint32_t get_peak_toffset() = 0;

    /*! Returns the latched peakInfo frequency offset (signed int32). */
    virtual int32_t get_peak_foffset() = 0;

    /*! Returns the number of peakInfoValid strobes seen so far. */
    virtual uint32_t get_peak_count() = 0;
};

}} // namespace rfnoc::oot_tum
