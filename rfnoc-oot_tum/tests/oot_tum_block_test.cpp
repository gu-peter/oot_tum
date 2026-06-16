//
// Copyright 2026 <author>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/rfnoc/actions.hpp>
#include <uhd/rfnoc/defaults.hpp>
#include <uhd/rfnoc/detail/graph.hpp>
#include <uhd/rfnoc/mock_block.hpp>
#include <uhd/rfnoc/mock_nodes.hpp>
#include <uhd/rfnoc/node_accessor.hpp>
#include <uhd/rfnoc/register_iface_holder.hpp>
#include <rfnoc/oot_tum/ofdm_tx_sl_block_control.hpp>
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <memory>

using namespace uhd::rfnoc;
using namespace rfnoc::oot_tum;
using namespace uhd::rfnoc::test;

// Redeclare this here, since it's only defined outside of UHD_API
noc_block_base::make_args_t::~make_args_t() = default;

constexpr size_t NUM_CHANS = 1;

/*
 * ofdm_tx_sl_block_fixture is a class which is instantiated before each test
 * case is run. It sets up the block container, ofdm_tx_sl_block_control
 * object and node accessor, all of which are accessible to the test case.
 * The instance of the object is destroyed at the end of each test case.
 */
struct ofdm_tx_sl_block_fixture
{
    ofdm_tx_sl_block_fixture()
        : block_container(
            get_mock_block(0xff2494ff, NUM_CHANS, NUM_CHANS, uhd::device_addr_t()))
        , test_ofdm_tx_sl(block_container.get_block<ofdm_tx_sl_block_control>())
    {
        node_accessor.init_props(test_ofdm_tx_sl.get());
    }

    mock_block_container block_container;
    std::shared_ptr<ofdm_tx_sl_block_control> test_ofdm_tx_sl;
    // The node_accessor is a C++ construct to bypass the public/private
    // division of the underlying C++ class. This should never be used in
    // production outside of unit tests, but here, it lets us peek inside the
    // class to verify it's working as expected.
    node_accessor_t node_accessor{};
};

/*
 * Verify that the block can be inserted into a graph and that its edge
 * types match what's defined in the block descriptor (ufix2 in, sc16 out).
 */
BOOST_FIXTURE_TEST_CASE(ofdm_tx_sl_test_edge_types, ofdm_tx_sl_block_fixture)
{
    detail::graph_t graph{};

    mock_terminator_t mock_source_term(NUM_CHANS);
    mock_terminator_t mock_sink_term(NUM_CHANS);

    constexpr size_t chan = 0;

    UHD_LOG_INFO("TEST", "Creating graph...");
    detail::graph_t::graph_edge_t edge_info{
        chan, chan, detail::graph_t::graph_edge_t::DYNAMIC, true};
    graph.connect(&mock_source_term, test_ofdm_tx_sl.get(), edge_info);
    graph.connect(test_ofdm_tx_sl.get(), &mock_sink_term, edge_info);

    UHD_LOG_INFO("TEST", "Committing graph...");
    graph.commit();
    UHD_LOG_INFO("TEST", "Commit complete.");
}

/*
 * Verify that set_enable()/get_enable() correctly poke/peek the
 * REG_ENABLE register.
 */
BOOST_FIXTURE_TEST_CASE(ofdm_tx_sl_test_enable, ofdm_tx_sl_block_fixture)
{
    test_ofdm_tx_sl->set_enable(true);
    BOOST_CHECK_EQUAL(
        block_container.reg_iface->write_memory.at(ofdm_tx_sl_block_control::REG_ENABLE),
        1u);

    test_ofdm_tx_sl->set_enable(false);
    BOOST_CHECK_EQUAL(
        block_container.reg_iface->write_memory.at(ofdm_tx_sl_block_control::REG_ENABLE),
        0u);

    block_container.reg_iface->read_memory[ofdm_tx_sl_block_control::REG_ENABLE] = 1;
    BOOST_CHECK(test_ofdm_tx_sl->get_enable());

    block_container.reg_iface->read_memory[ofdm_tx_sl_block_control::REG_ENABLE] = 0;
    BOOST_CHECK(!test_ofdm_tx_sl->get_enable());
}

/*
 * Verify that get_tx_payload_ready() correctly peeks the
 * REG_TX_PAYLOAD_READY register.
 */
BOOST_FIXTURE_TEST_CASE(ofdm_tx_sl_test_tx_payload_ready, ofdm_tx_sl_block_fixture)
{
    block_container.reg_iface->read_memory[ofdm_tx_sl_block_control::REG_TX_PAYLOAD_READY] =
        1;
    BOOST_CHECK(test_ofdm_tx_sl->get_tx_payload_ready());

    block_container.reg_iface->read_memory[ofdm_tx_sl_block_control::REG_TX_PAYLOAD_READY] =
        0;
    BOOST_CHECK(!test_ofdm_tx_sl->get_tx_payload_ready());
}
