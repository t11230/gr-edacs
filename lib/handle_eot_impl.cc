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

#include "handle_eot_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>

namespace {
constexpr size_t SAMPLES_PER_BATCH = (1024 / 4);
}

namespace gr {
namespace edacs {

handle_eot::sptr handle_eot::make(int samp_rate,
                                  int tone_freq,
                                  float tone_threshold,
                                  float noise_threshold)
{
    return gnuradio::get_initial_sptr(
        new handle_eot_impl(samp_rate, tone_freq, tone_threshold, noise_threshold));
}

/*
 * The private constructor
 */
handle_eot_impl::handle_eot_impl(int samp_rate,
                                 int tone_freq,
                                 float tone_threshold,
                                 float noise_threshold)
    : gr::sync_block("handle_eot",
                     gr::io_signature::make(1, 1, sizeof(float)),
                     gr::io_signature::make(1, 1, sizeof(float))),
      d_tone_threshold(tone_threshold),
      d_noise_threshold(noise_threshold),
      d_fft(samp_rate, SAMPLES_PER_BATCH, tone_freq)
{
    set_output_multiple(SAMPLES_PER_BATCH);
    set_max_noutput_items(SAMPLES_PER_BATCH);

    message_port_register_in(pmt::mp("status_in"));
    set_msg_handler(pmt::mp("status_in"),
                    boost::bind(&handle_eot_impl::change_status, this, _1));

    message_port_register_out(pmt::mp("status_out"));
}

void handle_eot_impl::set_tone_threshold(float tone_threshold)
{
    d_tone_threshold = tone_threshold;
}

void handle_eot_impl::set_noise_threshold(float noise_threshold)
{
    d_noise_threshold = noise_threshold;
}

int handle_eot_impl::get_sel_index() { return d_digital_assignment ? 1 : 0; }

void handle_eot_impl::change_status(pmt::pmt_t msg)
{
    d_digital_assignment = to_bool(msg);
    d_mute = false;

    // Wait for 2 batches of samples before muting again.
    // For example if SAMPLES_PER_BATCH = 1024, samp_rate = 48000, and DELAY = 2,
    // the wait is ~0.04s(SAMPLES_PER_BATCH * DELAY / samp_rate)

    d_channel_change_delay = 2;
}

void handle_eot_impl::notify_eot()
{
    pmt::pmt_t msg = pmt::from_bool(false);

    message_port_pub(pmt::mp("status_out"), msg);

    d_mute = true;
}

int handle_eot_impl::work(int noutput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    // First input stream
    const auto in = reinterpret_cast<const float*>(input_items[0]);

    // First output stream
    auto out = reinterpret_cast<float*>(output_items[0]);

    const auto mute_output = [&]() { memset(out, 0, sizeof(float) * noutput_items); };

    // Count down the delay for changing channels
    if (d_mute || d_channel_change_delay > 0) {

        --d_channel_change_delay;

        mute_output();

        return noutput_items;
    }

    // Ensure that the fft buffers are cleared
    d_fft.output();

    float max_noise_power = 0;

    for (size_t idx = 0; idx < noutput_items; ++idx) {
        const auto& item = in[idx];

        // Otherwise wait for the EOT tone
        d_fft.input(item);

        const auto curent_noise_power = d_iir.filter(item * item);
        if (curent_noise_power > max_noise_power) {
            max_noise_power = curent_noise_power;
        }
    }

    // If we didn't finish the fft don't do anything
    if (!d_fft.ready()) {
        return 0;
    }

    const auto tone_power = abs(d_fft.output());

    if (max_noise_power > d_noise_threshold) {

        std::cout << "Noisy channel, muting..." << std::endl;
        notify_eot();

    } else if (tone_power > d_tone_threshold) {

        std::cout << "End of transmission: " << std::endl;
        notify_eot();
    }

    if (d_mute) {
        mute_output();

    } else {

        // If we are not muted, just forward samples from in to out
        memcpy(out, in, sizeof(float) * noutput_items);
    }

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace edacs */
} /* namespace gr */
