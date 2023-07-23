/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 39.0.0 Wed Jul 19 15:42:39 2023.
 */

#ifndef CANTOOLS_H
#define CANTOOLS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


#ifndef EINVAL
#    define EINVAL 22
#endif

namespace cantools
{
/**
 * Signals in message Set_Input_Vel.
 *
 * All signal values are as on the CAN bus.
 */
struct odrive_set_input_vel_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float input_vel;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float input_torque_ff;
};

/**
 * Signals in message Set_Input_Steering.
 *
 * All signal values are as on the CAN bus.
 */
struct odrive_set_input_steering_t {
    /**
     * Range: [-steer_max_step, steer_max_step]
     * Scale: 1
     * Offset: 0
     */
    float input_steering;
};

/**
 * Signals in message Set_Controller_Mode.
 *
 * All signal values are as on the CAN bus.
 */
struct odrive_set_controller_mode_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t control_mode;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t input_mode;
};

/**
 * Pack message Set_Input_Vel.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int odrive_set_input_vel_pack(
    uint8_t *dst_p, 
    const struct odrive_set_input_vel_t *src_p,
    size_t size
);

/**
 * Unpack message Set_Input_Vel.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int odrive_set_input_vel_unpack(
    struct odrive_set_input_vel_t *dst_p,
    const uint8_t *src_p,
    size_t size
);

/**
 * Init message fields to default values from Set_Input_Vel.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int odrive_set_input_vel_init(struct odrive_set_input_vel_t *msg_p);


/**
 * Pack message Set_Input_Steering.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int odrive_set_input_steering_pack(
    uint8_t *dst_p, 
    const struct odrive_set_input_steering_t *src_p,
    size_t size
);

/**
 * Init message fields to default values from Set_Input_Steering.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int odrive_set_input_steering_init(struct odrive_set_input_steering_t *msg_p);

/**
 * Pack message Set_Controller_Mode.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int odrive_set_controller_mode_pack(
    uint8_t *dst_p,
    const struct odrive_set_controller_mode_t *src_p,
    size_t size);

/**
 * Init message fields to default values from Set_Controller_Mode.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int odrive_set_controller_mode_init(struct odrive_set_controller_mode_t *msg_p);

} // namespace cantools

#ifdef __cplusplus
}
#endif

#endif