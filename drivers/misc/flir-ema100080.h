/*
 * Copyright (C) 2022 TeledyneFlir
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#ifndef FLIR_EMA100080_H
#define FLIR_EMA100080_H


#define FLIR_EMA100080_COMPATIBLE_STR "flir,ema100080"
#define FLIR_EMA100080_MAX_I2C_CMDS 32
#define FLIR_EMA100080_NAME "flir-ema100080"

#define IOCTL_W(code, type) _IOW('c', code, type)
#define IOCTL_R(code, type) _IOR('c', code, type)
#define IOCTL_WR(code, type) _IOWR('c', code, type)
#define IOCTL_NWR(code) _IO('c', code)

#define IOCTL_FLIR_VF_PWR_ON_GET IOCTL_R(1, unsigned long)
#define IOCTL_FLIR_VF_PWR_ON IOCTL_W(2, unsigned long)
#define IOCTL_FLIR_VF_PWR_OFF IOCTL_W(3, unsigned long)
#endif /* FLIR_EMA100080_H */
