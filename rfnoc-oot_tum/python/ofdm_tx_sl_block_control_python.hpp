//
// Copyright 2026 <author>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/rfnoc/block_controller_factory_python.hpp>
#include <rfnoc/oot_tum/ofdm_tx_sl_block_control.hpp>

using namespace rfnoc::oot_tum;

void export_ofdm_tx_sl_block_control(py::module& m)
{
    py::class_<ofdm_tx_sl_block_control, uhd::rfnoc::noc_block_base, ofdm_tx_sl_block_control::sptr>(
        m, "ofdm_tx_sl_block_control")
        .def(py::init(
            &uhd::rfnoc::block_controller_factory<ofdm_tx_sl_block_control>::make_from))
        .def("get_tx_payload_ready", &ofdm_tx_sl_block_control::get_tx_payload_ready)
        .def("set_enable", &ofdm_tx_sl_block_control::set_enable, py::arg("enable"))
        .def("get_enable", &ofdm_tx_sl_block_control::get_enable)

        ;
}
