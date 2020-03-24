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

#ifndef INCLUDED_EDACS_PROC_MSG_IMPL_H
#define INCLUDED_EDACS_PROC_MSG_IMPL_H

#include <edacs/proc_msg.h>
#include <array>
#include <atomic>
#include <utility>

/* Number of possible EDACS channels */
#define MAX_CHANS 25
/* Length of a frame not including the frame sync bit sequence.
 * There are 2 messages, each with 3 40 bit copies */
#define PKT_LEN 240
#define ANALOG_CMD 0xEE
#define DIGITAL_CMD 0xEF
#define AGENCY_MASK 0x0700
#define FLEET_MASK 0x00F0
#define SUBFLEET_MASK 0x000F
/* Amount of characters that are output each frame */
#define OUTPUT_LEN 85
/* EDACS bits per second */
#define BIT_RATE 9600
/* Amount of samples to skip while waiting for a frequency shift to
 * take effect looking for the control channel. The samples have to
 * pass downstream from the signal source through many other blocks
 * until it reaches this block */
#define DELAY 960
/* Set to 1 to test the channel number finder (after finding all the
 * channel numbers, they will be reset and it will try to find them
 * again, looping like this forever */
#define TEST_CHAN_FINDER 0

namespace gr {
namespace edacs {

class proc_msg_impl : public proc_msg
{
private:
    std::vector<float> d_freq_list;
    float d_center_freq;

    struct control_message {
        control_message(uint64_t message_buffer);

        uint8_t cmd;
        uint8_t lcn;
        uint8_t status;
        uint8_t agency_id;
        uint8_t fleet_id;
        uint8_t subfleet_id;
        uint16_t ecc;

        uint16_t afs() const;
    };

    using FrameBuffer = std::array<uint8_t, PKT_LEN>;
    using CtrlMessagePair = std::pair<control_message, control_message>;

    void begin_frame();

    bool handle_frame_bit(bool bit);

    CtrlMessagePair parse_frame();

    bool filter_message(const control_message&);

    std::pair<bool, bool> filter_message_pair(const CtrlMessagePair&);

    int log_message(const control_message&, char*, int);

    int log_message_pair(const CtrlMessagePair&, char*, int);

    void process_message(const control_message&);

    bool d_in_frame{ false };
    size_t d_frame_buffer_bit_offset{ 0 };
    FrameBuffer d_frame_buffer;

    bool d_scanning{ true };
    int d_current_channel{ 0 };
    int d_control_channel{ 0 };

    size_t d_silent_bit_count{ 0 };
    size_t d_channel_change_delay{ 0 };

    uint8_t d_target_agency{ 0 };
    uint8_t d_target_fleet{ 0 };
    uint8_t d_target_subfleet{ 0 };

    bool d_enable_analog_voice{ true };
    bool d_enable_digital_voice{ true };

    std::atomic<int> d_current_voice_channel{ 0 };

public:
    proc_msg_impl(uint16_t talkgroup,
                  std::vector<float> freq_list,
                  float center_freq,
                  bool find_lcns,
                  bool analog,
                  bool digital);

    virtual ~proc_msg_impl() = default;

    /* Gets the channel number from msg_target so the Function Probe block
     * (and therefore any block) can access it. */
    int get_chan() { return 0; };

    /* Called by the message handler when the Handle EOT block notifies
     * us that a transmission has ended (and we should stop listening). */
    void change_eot_status(pmt::pmt_t msg);

    /* Called by the message handler when the Find Channel Number block
     * notifies us that it done scanning for active transmissions */
    void change_chan_status(pmt::pmt_t msg);

    /* Called before general_work() to make sure the number of input items
     * required to produce noutput_items is available. If the required number
     * of input items is not available, the scheduler halves nouput_items and
     * forecast() is called again. */
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    /* Called when there is new space in the output buffer or new input
     * available. Any required signal processing is done here. The number of
     * input items consumed is determined by calling consume() or
     * consume_each(). Returns a value <= noutput_items which is the number
     * of items produced. */
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace edacs
} // namespace gr

#endif /* INCLUDED_EDACS_PROC_MSG_IMPL_H */
