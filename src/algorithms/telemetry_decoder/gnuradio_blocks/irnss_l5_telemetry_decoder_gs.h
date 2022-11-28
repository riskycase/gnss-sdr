/*!
 * \file irnss_l5_telemetry_decoder_gs.h
 * \brief Interface of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based IRNSS receiver
 * \author Hrishikesh Patil, 2022. hrishikesh.patil(at)gmail.com
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_IRNSS_L5_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_IRNSS_L5_TELEMETRY_DECODER_GS_H
#include "IRNSS_L5.h"
#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "gnss_time.h"  // for timetags produced by Tracking
#include "irnss_navigation_message.h"
#include "nav_message_packet.h"
#include "tlm_conf.h"
#include "tlm_crc_stats.h"
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>  // for block
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <array>             // for array
#include <cstdint>           // for int32_t
#include <fstream>           // for ofstream
#include <memory>            // for std::unique_ptr
#include <string>            // for string

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks telemetry_decoder_gr_blocks
 * GNU Radio blocks for the demodulation of GNSS navigation messages.
 * \{ */


class irnss_l5_telemetry_decoder_gs;

using irnss_l5_telemetry_decoder_gs_sptr = gnss_shared_ptr<irnss_l5_telemetry_decoder_gs>;

irnss_l5_telemetry_decoder_gs_sptr irnss_l5_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf);

/*!
 * \brief This class implements a block that decodes the NAV data defined in IS-IRNSS-200M
 */
class irnss_l5_telemetry_decoder_gs : public gr::block
{
public:
    ~irnss_l5_telemetry_decoder_gs() override;
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        //!< Set receiver's channel
    void reset();

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) override;

private:
    friend irnss_l5_telemetry_decoder_gs_sptr irnss_l5_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        const Tlm_Conf &conf);

    irnss_l5_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf);

    bool irnss_word_parityCheck(uint32_t irnssword);
    bool decode_subframe(bool flag_invert);

    Irnss_Navigation_Message d_nav;
    Gnss_Satellite d_satellite;
    Nav_Message_Packet d_nav_msg_packet;
    std::unique_ptr<Tlm_CRC_Stats> d_Tlm_CRC_Stats;

    std::array<int32_t, IRNSS_PREAMBLE_LENGTH_BITS> d_preamble_samples{};

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    boost::circular_buffer<float> d_symbol_history;

    uint64_t d_sample_counter;
    uint64_t d_preamble_index;
    uint64_t d_last_valid_preamble;

    int32_t d_bits_per_preamble;
    int32_t d_samples_per_preamble;
    int32_t d_preamble_period_symbols;
    int32_t d_CRC_error_counter;
    int32_t d_channel;

    uint32_t d_required_symbols;
    uint32_t d_prev_IRNSS_frame_4bytes;
    uint32_t d_max_symbols_without_valid_frame;
    uint32_t d_stat;
    uint32_t d_TOW_at_Preamble_ms;
    uint32_t d_TOW_at_current_symbol_ms;

    bool d_flag_frame_sync;
    bool d_flag_preamble;
    bool d_sent_tlm_failed_msg;
    bool d_flag_PLL_180_deg_phase_locked;
    bool d_flag_TOW_set;
    bool d_dump;
    bool d_dump_mat;
    bool d_remove_dat;
    bool d_enable_navdata_monitor;
    bool d_dump_crc_stats;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_IRNSS_L5_TELEMETRY_DECODER_GS_H
