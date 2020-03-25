/* -*- c++ -*- */
/*
 * Copyright 2017 Clayton Caron.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "proc_msg_impl.h"
#include <gnuradio/io_signature.h>
#include <iostream>
#include <thread>

namespace {

// Conversion factor from frequency number (801.2) to MHz
constexpr float MHZ_SCALE = 1e6;

// Maximum and minumum valid channel values
constexpr auto MIN_EDACS_CHANNEL = 1;
constexpr auto MAX_EDACS_CHANNEL = 25;

// Shift masks for extracting AFS from talkgroup
constexpr auto AGENCY_MASK = 0x0700;
constexpr auto FLEET_MASK = 0x00F0;
constexpr auto SUBFLEET_MASK = 0x000F;

// Bitrate for EDACS control channel in bits / second
constexpr auto EDACS_CONTROL_BITRATE = 9600;

// Amount of samples to skip while waiting for a frequency shift to
// take effect looking for the control channel. The samples have to
// pass downstream from the signal source through many other blocks
// until it reaches this block
constexpr auto CHANNEL_CHANGE_DELAY = 960;

// Basic commands that we support
constexpr uint8_t ANALOG_VOICE_ASSIGN_CMD = 0xEE;
constexpr uint8_t DIGITAL_VOICE_ASSIGN_CMD = 0xEF;

} // namespace

namespace gr {
namespace edacs {

proc_msg::sptr proc_msg::make(uint16_t talkgroup,
                              std::vector<float> freq_list,
                              float center_freq,
                              bool find_lcns,
                              bool enable_analog_voice,
                              bool enable_digital_voice)
{
    return gnuradio::get_initial_sptr(new proc_msg_impl(talkgroup,
                                                        freq_list,
                                                        center_freq,
                                                        find_lcns,
                                                        enable_analog_voice,
                                                        enable_digital_voice));
}

/* The private constructor */
proc_msg_impl::proc_msg_impl(uint16_t talkgroup,
                             std::vector<float> freq_list,
                             float center_freq,
                             bool find_lcns,
                             bool enable_analog_voice,
                             bool enable_digital_voice)
    : gr::block("proc_msg",
                gr::io_signature::make(1, 1, sizeof(unsigned char)),
                gr::io_signature::make(1, 1, sizeof(unsigned char))),
      d_center_freq(center_freq * MHZ_SCALE),
      d_enable_analog_voice(enable_analog_voice),
      d_enable_digital_voice(enable_digital_voice)
{
    d_freq_list.reserve(freq_list.size());

    for (const auto& freq : freq_list) {
        d_freq_list.push_back(freq * MHZ_SCALE);
    }

    message_port_register_in(pmt::mp("eot_status_in"));
    set_msg_handler(pmt::mp("eot_status_in"),
                    boost::bind(&proc_msg_impl::change_eot_status, this, _1));

    message_port_register_in(pmt::mp("chan_status_in"));
    set_msg_handler(pmt::mp("chan_status_in"),
                    boost::bind(&proc_msg_impl::change_chan_status, this, _1));

    message_port_register_out(pmt::mp("eot_status_out"));
    message_port_register_out(pmt::mp("chan_status_out"));
    message_port_register_out(pmt::mp("ctrl_freq"));
    message_port_register_out(pmt::mp("voice_freq"));

    d_target_agency = (AGENCY_MASK & talkgroup) >> 8;
    d_target_fleet = (FLEET_MASK & talkgroup) >> 4;
    d_target_subfleet = (SUBFLEET_MASK & talkgroup);

    // Figure out how large our output items will be
    char tmp_buffer[1024];
    d_output_item_size =
        log_message_pair({ { 0 }, { 0 } }, tmp_buffer, sizeof(tmp_buffer));

    set_output_multiple(d_output_item_size);

    std::cout << "Output size: " << d_output_item_size << " bytes" << std::endl;
}

void proc_msg_impl::change_eot_status(pmt::pmt_t msg)
{
    if (!to_bool(msg)) {
        d_current_voice_channel = 0;
    }
}

void proc_msg_impl::change_chan_status(pmt::pmt_t msg)
{
    // TODO: Get this working again
    // if (d_find_lcns) {
    //     lf_lcn = true;
    //     int unknown_count = 0;
    //     int i;
    //     /* Check if all channel number have been found */
    //     for (i = 0; i < n_chans; i++) {
    //         chan_indices[i] = to_long(pmt::vector_ref(msg, i));
    //         if (chan_indices[i] == -1)
    //             unknown_count++;
    //     }
    //     /* All channel numbers have been found, so print out the proper
    //      * frequency order and sort d_freq_list */
    //     if (unknown_count <= 1 && !TEST_CHAN_FINDER) {
    //         for (i = 0; i < n_chans; i++) {
    //             if (chan_indices[i] == -1)
    //                 temp_freqs[i] = d_freq_list[ctrl_chan - 1];
    //             else
    //                 temp_freqs[i] = d_freq_list[chan_indices[i]];
    //         }
    //         printf("Frequency Order:\n");
    //         for (i = 0; i < n_chans - 1; i++) {
    //             d_freq_list[i] = temp_freqs[i];
    //             printf("%.4f, ", d_freq_list[i]);
    //         }
    //         d_freq_list[n_chans - 1] = temp_freqs[n_chans - 1];
    //         printf("%.4f\n", d_freq_list[n_chans - 1]);
    //         d_find_lcns = false;
    //         lf_lcn = false;
    //     }
    // }
}

proc_msg_impl::control_message::control_message(uint64_t message_buffer)
    : cmd((message_buffer >> 32) & 0xFF),
      lcn(static_cast<uint8_t>((message_buffer) >> 27) & 0x1F),
      status(static_cast<uint8_t>((message_buffer) >> 23) & 0x0F),
      agency_id(static_cast<uint8_t>((message_buffer) >> 20) & 0x07),
      fleet_id(static_cast<uint8_t>((message_buffer) >> 16) & 0xF),
      subfleet_id(static_cast<uint8_t>((message_buffer) >> 12) & 0xF),
      ecc(static_cast<uint16_t>((message_buffer) >> 0) & 0xFFF)
{
}

uint16_t proc_msg_impl::control_message::afs() const
{
    return (agency_id << 8) | (fleet_id << 4) | subfleet_id;
}

void proc_msg_impl::begin_frame()
{
    d_in_frame = true;

    // TODO: There appears to be a missing bit at the
    // beginning of the stream. Not sure why
    d_frame_buffer_bit_offset = 1;

    // Indicate that we have heard valid bits
    d_silent_bit_count = 0;
}

bool proc_msg_impl::handle_frame_bit(bool bit)
{
    if (d_frame_buffer_bit_offset >= sizeof(d_frame_buffer)) {
        return false;
    }

    d_frame_buffer[d_frame_buffer_bit_offset] = bit ? 1 : 0;

    ++d_frame_buffer_bit_offset;

    // Have we filled a complete frame?
    if (d_frame_buffer_bit_offset < sizeof(d_frame_buffer)) {
        return false;
    }

    // The current frame is done, so start looking for the next one
    d_in_frame = false;

    return true;
}

proc_msg_impl::CtrlMessagePair proc_msg_impl::parse_frame()
{
    constexpr size_t MESSAGE_SIZE = 40;

    constexpr size_t MESSAGE_1_BASE = MESSAGE_SIZE * 3 * 0;
    constexpr size_t MESSAGE_2_BASE = MESSAGE_SIZE * 3 * 1;

    constexpr size_t COPY_1_BASE = MESSAGE_SIZE * 0;
    constexpr size_t COPY_2_BASE = MESSAGE_SIZE * 1;
    constexpr size_t COPY_3_BASE = MESSAGE_SIZE * 2;

    uint64_t message_1_buffer = 0;
    uint64_t message_2_buffer = 0;

    for (size_t message_offset = 0; message_offset < MESSAGE_SIZE; ++message_offset) {

        uint8_t message_1_bit_accumulator = 0;
        uint8_t message_2_bit_accumulator = 0;

        message_1_bit_accumulator +=
            (d_frame_buffer[MESSAGE_1_BASE + COPY_1_BASE + message_offset]) & 1;
        message_2_bit_accumulator +=
            (d_frame_buffer[MESSAGE_2_BASE + COPY_1_BASE + message_offset]) & 1;

        message_1_bit_accumulator +=
            (~d_frame_buffer[MESSAGE_1_BASE + COPY_2_BASE + message_offset]) & 1;
        message_2_bit_accumulator +=
            (~d_frame_buffer[MESSAGE_2_BASE + COPY_2_BASE + message_offset]) & 1;

        message_1_bit_accumulator +=
            (d_frame_buffer[MESSAGE_1_BASE + COPY_3_BASE + message_offset]) & 1;
        message_2_bit_accumulator +=
            (d_frame_buffer[MESSAGE_2_BASE + COPY_3_BASE + message_offset]) & 1;

        message_1_buffer |= static_cast<uint64_t>(message_1_bit_accumulator >= 2 ? 1 : 0)
                            << (MESSAGE_SIZE - message_offset);
        message_2_buffer |= static_cast<uint64_t>(message_2_bit_accumulator >= 2 ? 1 : 0)
                            << (MESSAGE_SIZE - message_offset);
    }

    // Construct 2 messages and return them
    return { message_1_buffer, message_2_buffer };
}

int proc_msg_impl::log_message(const control_message& msg,
                               char* log_buffer,
                               int log_buffer_size)
{
    return snprintf(
        log_buffer,
        log_buffer_size,
        "{ \"command\": \"0x%02x\", \"channel\": \"%2d\", \"talkgroup\": \"%4d\"}\n",
        msg.cmd,
        msg.lcn,
        msg.afs());
}

int proc_msg_impl::log_message_pair(const CtrlMessagePair& msg_pair,
                                    char* log_buffer,
                                    int log_buffer_size)
{
    auto written = log_message(msg_pair.first, log_buffer, log_buffer_size);

    // Check for truncated output and add a NULL terminator if needed
    if (written >= log_buffer_size) {

        log_buffer[log_buffer_size - 1] = '\x00';

        // At this point we have completely filled the buffer, so just return early
        return log_buffer_size;
    }

    // Now we will overwrite the NULL byte from the above snprintf and add
    // the log string for the second message

    const auto remaining = log_buffer_size - written;

    written += log_message(msg_pair.second, &log_buffer[written], remaining);

    // Add 1 to account the the NULL terminator added by snprintf
    written += 1;

    // Check for truncated output and add a NULL terminator if needed
    if (written >= log_buffer_size) {

        log_buffer[log_buffer_size - 1] = '\x00';

        written = log_buffer_size;
    }

    return written;
}

bool proc_msg_impl::filter_message(const control_message& msg)
{
    // Filter out voice types that we are not interested in
    if ((msg.cmd == ANALOG_VOICE_ASSIGN_CMD && !d_enable_analog_voice) ||
        (msg.cmd == DIGITAL_VOICE_ASSIGN_CMD && !d_enable_digital_voice)) {
        return false;
    }

    // Sanity checking on the channel number
    if (msg.lcn < MIN_EDACS_CHANNEL || msg.lcn > MAX_EDACS_CHANNEL) {
        return false;
    }

    // If we are currently listening ignore it
    if (d_current_voice_channel) {
        return false;
    }

    // TODO: Add channel finding logic back

    // Check agency id
    if (d_target_agency && msg.agency_id && msg.agency_id != d_target_agency) {
        return false;
    }

    // Check fleet id
    if (d_target_fleet && msg.fleet_id && msg.fleet_id != d_target_fleet) {
        return false;
    }

    // Check subfleet id
    if (d_target_subfleet && msg.subfleet_id && msg.subfleet_id != d_target_subfleet) {
        return false;
    }

    // At this point our message is valid and matches our filters
    return true;
}

void proc_msg_impl::process_message(const control_message& msg)
{
    // Build notifications for the EOT block
    pmt::pmt_t out_msg;

    if (msg.cmd == DIGITAL_VOICE_ASSIGN_CMD) {
        printf("Digital voice channel assignment: %2d "
               "(Agency %2d, Fleet %2d, Subfleet %2d)\n",
               msg.lcn,
               msg.agency_id,
               msg.fleet_id,
               msg.subfleet_id);

        out_msg = pmt::from_bool(true);

    } else if (msg.cmd == ANALOG_VOICE_ASSIGN_CMD) {
        printf("Analog voice channel assignment: %2d "
               "(Agency %2d, Fleet %2d, Subfleet %2d)\n",
               msg.lcn,
               msg.agency_id,
               msg.fleet_id,
               msg.subfleet_id);

        out_msg = pmt::from_bool(false);
    } else {
        // Other message types not supported
        return;
    }

    message_port_pub(pmt::mp("eot_status_out"), out_msg);

    // Now change our listening channel and update the tuned frequency
    d_current_voice_channel = msg.lcn;

    const auto freq_offset = lcn_to_offset(d_current_voice_channel);

    message_port_pub(pmt::mp("voice_freq"), pmt::from_double(freq_offset));
}

float proc_msg_impl::lcn_to_frequency(int lcn)
{
    if (lcn < 1 || lcn > d_freq_list.size()) {

        std::cout << "Invalid LCN: [" << std::dec << lcn << "]" << std::endl;

        return 0;
    }

    return d_freq_list[lcn - 1];
}

float proc_msg_impl::lcn_to_offset(int lcn)
{
    return d_center_freq - lcn_to_frequency(lcn);
}

void proc_msg_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    constexpr auto FRAME_LEN = std::tuple_size<FrameBuffer>::value;

    if (ninput_items_required.empty()) {
        return;
    }

    // The number of input items required to produce noutput_items
    ninput_items_required[0] = (noutput_items / d_output_item_size - 1) * FRAME_LEN + 1;
}

int proc_msg_impl::general_work(int noutput_items,
                                gr_vector_int& ninput_items,
                                gr_vector_const_void_star& input_items,
                                gr_vector_void_star& output_items)
{
    // First input stream
    const auto in = reinterpret_cast<const uint8_t*>(input_items[0]);
    const auto in_size = ninput_items[0];

    // First output stream
    auto out = reinterpret_cast<char*>(output_items[0]);

    // Track how many output items we have produced
    int produced = 0;

    // Process each bit that is provided
    for (size_t idx = 0; idx < in_size; ++idx) {
        const auto& current_item = in[idx];

        // Check if we have a pattern that matches the access code
        if (current_item & 0x02) {

            // In this case we have found the start of a frame
            begin_frame();

            if (d_scanning) {

                std::cout << "Control channel found [" << d_control_channel << "]"
                          << std::endl;

                d_scanning = false;

                // TODO: Add channel finding back
            }
        }

        // Currently reading a frame off the wire
        if (d_in_frame) {

            // Accumulate bits and check if we have a complete frame
            if (!handle_frame_bit((current_item & 1) == 1)) {

                // If not, grab another bit from the wire
                continue;
            }

            // Otherwise, parse the current frame

            const auto message_pair = parse_frame();

            const auto remaining = noutput_items - produced;

            produced += log_message_pair(message_pair, &out[produced], remaining);

            if (filter_message(message_pair.first)) {
                process_message(message_pair.first);
            }

            if (filter_message(message_pair.second)) {
                process_message(message_pair.second);
            }
        }

        // As long as we are not scanning we should be receiving data
        if (!d_scanning) {

            // If one second has gone by without getting a packet, assume
            // the control channel has changed and do another scan
            if (d_silent_bit_count > EDACS_CONTROL_BITRATE) {

                std::cout << "No data received, Scanning for new control channel"
                          << std::endl;

                d_silent_bit_count = 0;
                d_scanning = true;
                break;

            } else {
                ++d_silent_bit_count;
            }
        }
    }

    if (d_scanning) {

        // Check if we are waiting for a channel change to take effect

        if (d_channel_change_delay > 0) {

            --d_channel_change_delay;

        } else {

            // If not, go ahead and change it now

            // Handle case where we have not tuned to a control channel yet
            // or we have exceeded the number of frequencies available
            if (!d_control_channel || d_control_channel > d_freq_list.size()) {

                d_control_channel = 1;

            } else {

                ++d_control_channel;
            }

            // Set up expected delay for channel change to take effect
            d_channel_change_delay = CHANNEL_CHANGE_DELAY;

            std::cout << "Looking for control channel on [" << d_control_channel << "]"
                      << std::endl;

            auto msg = pmt::from_double(lcn_to_offset(d_control_channel));

            message_port_pub(pmt::mp("ctrl_freq"), msg);
        }
    }

    /* Tell runtime system how many input items we consumed on
     * each input stream. */
    consume_each(in_size);

    /* Tell runtime system how many output items we produced. */
    return produced;
}

} /* namespace edacs */
} /* namespace gr */
