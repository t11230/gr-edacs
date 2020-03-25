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

#ifndef INCLUDED_EDACS_HANDLE_EOT_IMPL_H
#define INCLUDED_EDACS_HANDLE_EOT_IMPL_H

#include <gnuradio/fft/goertzel.h>
#include <gnuradio/filter/single_pole_iir.h>
#include <edacs/handle_eot.h>

namespace gr {
namespace edacs {

class handle_eot_impl : public handle_eot
{
private:
    // If we are not muted, listen for the end of transmission
    // tone. This is done using the Goertzel algorithm which is
    // essentially an FFT for computing a single spectral bin.
    // For EDACS this EOT tone is at 4800 Hz, so if the amplitude
    // at this frequency is found to be greater than our defined
    // threshold, we mute
    fft::goertzel d_fft;

    // In case we are sent to a frequency with no voice or we somehow
    // miss the EOT tone sequence, try to detect static and correct by
    // muting
    filter::single_pole_iir<double, double, double> d_iir;

    bool d_mute{ true };
    size_t d_channel_change_delay{ 0 };
    bool d_digital_assignment{ false };
    float d_tone_threshold;
    float d_noise_threshold;

    void notify_eot();

public:
    handle_eot_impl(int samp_rate,
                    int tone_freq,
                    float tone_threshold,
                    float noise_threshold);

    virtual ~handle_eot_impl() = default;

    /* Getters & setters */
    void set_tone_threshold(float tone_threshold);
    void set_noise_threshold(float noise_threshold);
    int get_sel_index();

    /* Called by the message handler when the Process Message block notifies
     * us that a transmission has started (and we should stop muting). */
    void change_status(pmt::pmt_t msg);

    /* Signal processing happens here. Note that this is a sync block so
     * noutput_items always equals the number of input items. */
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace edacs
} // namespace gr

#endif /* INCLUDED_EDACS_HANDLE_EOT_IMPL_H */
