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

#include <string.h>

#include "pegasus_control/cantools.hpp"

namespace cantools
{
static inline uint8_t pack_left_shift_u8(uint8_t value, uint8_t shift, uint8_t mask)
{
  return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(uint16_t value, uint8_t shift, uint8_t mask)
{
  return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u32(uint32_t value, uint8_t shift, uint8_t mask)
{
  return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(uint16_t value, uint8_t shift, uint8_t mask)
{
  return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t pack_right_shift_u32(uint32_t value, uint8_t shift, uint8_t mask)
{
  return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(uint8_t value, uint8_t shift, uint8_t mask)
{
  return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint32_t unpack_left_shift_u32(uint8_t value, uint8_t shift, uint8_t mask)
{
  return (uint32_t)((uint32_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(uint8_t value, uint8_t shift, uint8_t mask)
{
  return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(uint8_t value, uint8_t shift, uint8_t mask)
{
  return (uint16_t)((uint16_t)(value & mask) >> shift);
}

static inline uint32_t unpack_right_shift_u32(uint8_t value, uint8_t shift, uint8_t mask)
{
  return (uint32_t)((uint32_t)(value & mask) >> shift);
}

int odrive_set_input_steering_init(struct cantools::odrive_set_input_steering_t* msg_p)
{
  if (msg_p == NULL)
    return -1;

  memset(msg_p, 0, sizeof(struct cantools::odrive_set_input_steering_t));

  return 0;
}

int odrive_set_input_steering_pack(uint8_t* dst_p, const struct cantools::odrive_set_input_steering_t* src_p,
                                   size_t size)
{
  uint32_t input_steering;

  if (size < 8u)
  {
    return (-EINVAL);
  }

  memset(&dst_p[0], 0, 8);

  memcpy(&input_steering, &src_p->input_steering, sizeof(input_steering));
  dst_p[0] |= pack_left_shift_u32(input_steering, 0u, 0xffu);
  dst_p[1] |= pack_right_shift_u32(input_steering, 8u, 0xffu);

  return (8);
}

int odrive_set_input_vel_init(struct cantools::odrive_set_input_vel_t* msg_p)
{
  if (msg_p == NULL)
    return -1;

  memset(msg_p, 0, sizeof(struct cantools::odrive_set_input_vel_t));

  return 0;
}

int odrive_set_input_vel_pack(uint8_t* dst_p, const struct cantools::odrive_set_input_vel_t* src_p, size_t size)
{
  uint32_t input_torque_ff;
  uint32_t input_vel;

  if (size < 8u)
  {
    return (-EINVAL);
  }

  memset(&dst_p[0], 0, 8);

  memcpy(&input_vel, &src_p->input_vel, sizeof(input_vel));
  dst_p[0] |= pack_left_shift_u32(input_vel, 0u, 0xffu);
  dst_p[1] |= pack_right_shift_u32(input_vel, 8u, 0xffu);
  dst_p[2] |= pack_right_shift_u32(input_vel, 16u, 0xffu);
  dst_p[3] |= pack_right_shift_u32(input_vel, 24u, 0xffu);
  memcpy(&input_torque_ff, &src_p->input_torque_ff, sizeof(input_torque_ff));
  dst_p[4] |= pack_left_shift_u32(input_torque_ff, 0u, 0xffu);
  dst_p[5] |= pack_right_shift_u32(input_torque_ff, 8u, 0xffu);
  dst_p[6] |= pack_right_shift_u32(input_torque_ff, 16u, 0xffu);
  dst_p[7] |= pack_right_shift_u32(input_torque_ff, 24u, 0xffu);

  return (8);
}

int odrive_set_input_vel_unpack(struct cantools::odrive_set_input_vel_t* dst_p, const uint8_t* src_p, size_t size)
{
  uint32_t input_torque_ff;
  uint32_t input_vel;

  if (size < 8u)
  {
    return (-EINVAL);
  }

  input_vel = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
  input_vel |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
  input_vel |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
  input_vel |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);
  memcpy(&dst_p->input_vel, &input_vel, sizeof(dst_p->input_vel));
  input_torque_ff = unpack_right_shift_u32(src_p[4], 0u, 0xffu);
  input_torque_ff |= unpack_left_shift_u32(src_p[5], 8u, 0xffu);
  input_torque_ff |= unpack_left_shift_u32(src_p[6], 16u, 0xffu);
  input_torque_ff |= unpack_left_shift_u32(src_p[7], 24u, 0xffu);
  memcpy(&dst_p->input_torque_ff, &input_torque_ff, sizeof(dst_p->input_torque_ff));

  return (0);
}

int odrive_set_controller_mode_pack(uint8_t* dst_p, const struct cantools::odrive_set_controller_mode_t* src_p,
                                    size_t size)
{
  if (size < 8u)
  {
    return (-EINVAL);
  }

  memset(&dst_p[0], 0, 8);

  dst_p[0] |= pack_left_shift_u32(src_p->control_mode, 0u, 0xffu);
  dst_p[1] |= pack_right_shift_u32(src_p->control_mode, 8u, 0xffu);
  dst_p[2] |= pack_right_shift_u32(src_p->control_mode, 16u, 0xffu);
  dst_p[3] |= pack_right_shift_u32(src_p->control_mode, 24u, 0xffu);
  dst_p[4] |= pack_left_shift_u32(src_p->input_mode, 0u, 0xffu);
  dst_p[5] |= pack_right_shift_u32(src_p->input_mode, 8u, 0xffu);
  dst_p[6] |= pack_right_shift_u32(src_p->input_mode, 16u, 0xffu);
  dst_p[7] |= pack_right_shift_u32(src_p->input_mode, 24u, 0xffu);

  return (8);
}

int odrive_set_controller_mode_init(struct odrive_set_controller_mode_t* msg_p)
{
  if (msg_p == NULL)
    return -1;

  memset(msg_p, 0, sizeof(struct odrive_set_controller_mode_t));

  return 0;
}

int odrive_get_encoder_estimates_unpack(struct cantools::odrive_get_encoder_estimates_t* dst_p, const uint8_t* src_p,
                                        size_t size)
{
  uint32_t pos_estimate;
  uint32_t vel_estimate;

  if (size < 8u)
  {
    return (-EINVAL);
  }

  pos_estimate = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
  pos_estimate |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
  pos_estimate |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
  pos_estimate |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);
  memcpy(&dst_p->pos_estimate, &pos_estimate, sizeof(dst_p->pos_estimate));
  vel_estimate = unpack_right_shift_u32(src_p[4], 0u, 0xffu);
  vel_estimate |= unpack_left_shift_u32(src_p[5], 8u, 0xffu);
  vel_estimate |= unpack_left_shift_u32(src_p[6], 16u, 0xffu);
  vel_estimate |= unpack_left_shift_u32(src_p[7], 24u, 0xffu);
  memcpy(&dst_p->vel_estimate, &vel_estimate, sizeof(dst_p->vel_estimate));

  return (0);
}

int odrive_get_encoder_estimates_init(struct cantools::odrive_get_encoder_estimates_t* msg_p)
{
  if (msg_p == NULL)
    return -1;

  memset(msg_p, 0, sizeof(struct cantools::odrive_get_encoder_estimates_t));

  return 0;
}

}  // namespace cantools
