//
// Copyright 2026 <author>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/rfnoc/block_controller_factory_python.hpp>
#include <rfnoc/oot_tum/ofdm_rx_sl_block_control.hpp>

using namespace rfnoc::oot_tum;

void export_ofdm_rx_sl_block_control(py::module& m)
{
    py::class_<ofdm_rx_sl_block_control, uhd::rfnoc::noc_block_base, ofdm_rx_sl_block_control::sptr>(
        m, "ofdm_rx_sl_block_control")
        .def(py::init(
            &uhd::rfnoc::block_controller_factory<ofdm_rx_sl_block_control>::make_from))
        .def("set_start", &ofdm_rx_sl_block_control::set_start, py::arg("start"))
        .def("get_start", &ofdm_rx_sl_block_control::get_start)
        .def("set_freq_correction_en",
            &ofdm_rx_sl_block_control::set_freq_correction_en, py::arg("enable"))
        .def("get_freq_correction_en", &ofdm_rx_sl_block_control::get_freq_correction_en)
        .def("get_peak_correlation", &ofdm_rx_sl_block_control::get_peak_correlation)
        .def("get_peak_threshold", &ofdm_rx_sl_block_control::get_peak_threshold)
        .def("get_peak_toffset", &ofdm_rx_sl_block_control::get_peak_toffset)
        .def("get_peak_foffset", &ofdm_rx_sl_block_control::get_peak_foffset)
        .def("get_peak_count", &ofdm_rx_sl_block_control::get_peak_count)

        ;
}
