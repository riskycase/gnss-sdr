/*!
 * \file irnss_l5_telemetry_decoder.h
 * \brief Interface of an adapter of a IRNSS L5 NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Hrishikesh Patil, 2022. hrishikeshpatil.754(at)gmail.com
 *
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


#ifndef GNSS_SDR_IRNSS_L5_TELEMETRY_DECODER_H
#define GNSS_SDR_IRNSS_L5_TELEMETRY_DECODER_H

#include "gnss_satellite.h"  // for Gnss_Satellite
#include "gnss_synchro.h"
#include "irnss_l5_telemetry_decoder_gs.h"
#include "telemetry_decoder_interface.h"
#include "tlm_conf.h"
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>                   // for size_t
#include <string>

/** \addtogroup Telemetry_Decoder Telemetry Decoder
 * Classes for the decoding of GNSS Navigation messages.
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters telemetry_decoder_adapters
 * Wrap GNU Radio blocs for the decoding of GNSS Navigation messages with a
 * TelemetryDecoderInterface
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for IRNSS L5
 */
class IrnssL5TelemetryDecoder : public TelemetryDecoderInterface
{
public:
    IrnssL5TelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite& satellite) override;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "IRNSS_L5_Telemetry_Decoder"
    inline std::string implementation() override
    {
        return "IRNSS_L5_Telemetry_Decoder";
    }

    inline void set_channel(int channel) override { telemetry_decoder_->set_channel(channel); }

    inline void reset() override
    {
        telemetry_decoder_->reset();
    }

    inline size_t item_size() override
    {
        return sizeof(Gnss_Synchro);
    }

private:
    irnss_l5_telemetry_decoder_gs_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    Tlm_Conf tlm_parameters_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_IRNSS_L5_TELEMETRY_DECODER_H
