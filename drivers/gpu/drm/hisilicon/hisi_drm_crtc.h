/*
 * Hisilicon Terminal SoCs drm driver
 *
 * Copyright (c) 2014-2015 Hisilicon Limited.
 * Author: Xinwei Kong <kong.kongxinwei@hisilicon.com> for hisilicon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __HISI_DRM_CRTC_H__
#define __HISI_DRM_CRTC_H__

#define to_hisi_crtc(crtc)		container_of(crtc, struct hisi_crtc, base)
#define to_hisi_crtc_state(state)	container_of(state, struct hisi_crtc_state, base)

struct hisi_crtc {
	struct drm_crtc base;
	void *ops;
	void *ctx;
	bool enable;
};

struct hisi_crtc_ops {
	void (*disable)(struct hisi_crtc *hcrtc);
	void (*enable)(struct hisi_crtc *hcrtc);
	void (*mode_prepare)(struct hisi_crtc *hcrtc);
	bool (*mode_fixup)(struct hisi_crtc *hcrtc,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adj_mode);
	void (*mode_set_nofb)(struct hisi_crtc *hcrtc);
	void (*atomic_begin)(struct hisi_crtc *hcrtc);
	void (*atomic_flush)(struct hisi_crtc *hcrtc);
	void (*destroy)(struct hisi_crtc *hcrtc);
	irqreturn_t (*irq_handler)(int irq, struct hisi_crtc *hcrtc);
	int (*enable_vblank)(struct hisi_crtc *hcrtc);
	void (*disable_vblank)(struct hisi_crtc *hcrtc);
	int (*install_properties)(struct drm_device *dev,
				  struct hisi_crtc *hcrtc);
};

enum composotion_type {
	COMPOSITION_UNKNOWN	= 0,
	COMPOSITION_GLES	= 1,
	COMPOSITION_HWC		= 2,
	COMPOSITION_MIXED	= 3
};

struct hisi_crtc_state {
	struct drm_crtc_state base;
	u8 comp_type;
};

extern int hisi_drm_crtc_init(struct drm_device *dev,
		       struct hisi_crtc *crtc,
		       struct drm_plane *plane);

extern int hisi_drm_crtc_enable_vblank(struct drm_device *dev, int c);
extern void hisi_drm_crtc_disable_vblank(struct drm_device *dev, int c);
extern int hisi_drm_crtc_irq_install(struct drm_device *dev, int irq,
				     unsigned long flags, void *data);
#endif
