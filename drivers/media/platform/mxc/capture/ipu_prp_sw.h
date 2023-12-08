/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2004-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2019 NXP
 */

/*!
 * @file ipu_prp_sw.h
 *
 * @brief This file contains the IPU PRP use case driver header.
 *
 * @ingroup IPU
 */

#ifndef _INCLUDE_IPU__PRP_SW_H_
#define _INCLUDE_IPU__PRP_SW_H_

int csi_enc_select(void *private);
int csi_enc_deselect(void *private);
int prp_enc_select(void *private);
int prp_enc_deselect(void *private);

#if  defined(CONFIG_MXC_IPU_DEVICE_QUEUE_SDC) || defined(CONFIG_FLIR_FB_VIDEOFLOW)
int foreground_sdc_select(void *private);
int foreground_sdc_deselect(void *private);
int bg_overlay_sdc_select(void *private);
int bg_overlay_sdc_deselect(void *private);
#endif

#if defined(CONFIG_MXC_IPU_PRP_ENC) || defined(CONFIG_MXC_IPU_CSI_ENC)
int prp_still_select(void *private);
int prp_still_deselect(void *private);
#endif

#ifdef CONFIG_FLIR_FB_VIDEOFLOW
int prp_vf_sdc_select(void *private);
int prp_vf_sdc_deselect(void *private);
int prp_vf_sdc_select_bg(void *private);
int prp_vf_sdc_deselect_bg(void *private);
int prp_vf_flir_select(void *private);
int prp_vf_flir_deselect(void *private);
int prp_vf_ninjago_select(void *private);
int prp_vf_ninjago_deselect(void *private);
#endif
#endif
