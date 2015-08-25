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

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_drm_drv.h"
#include "hisi_drm_plane.h"
#include "hisi_drm_crtc.h"


int hisi_drm_crtc_enable_vblank(struct drm_device *dev, int c)
{
	struct drm_crtc *crtc = drm_get_crtc_from_index(dev, c);
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;
	int ret = 0;

	if (ops->enable_vblank)
		ret = ops->enable_vblank(hcrtc);

	return ret;
}

void hisi_drm_crtc_disable_vblank(struct drm_device *dev, int c)
{
	struct drm_crtc *crtc = drm_get_crtc_from_index(dev, c);
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->disable_vblank)
		ops->disable_vblank(hcrtc);

}

irqreturn_t hisi_drm_crtc_irq_handler(int irq, void *arg)
{
	struct hisi_crtc *hcrtc = (struct hisi_crtc *)arg;
	struct hisi_crtc_ops *ops = hcrtc->ops;
	irqreturn_t ret = IRQ_HANDLED;

	if (ops->irq_handler)
		ret = ops->irq_handler(irq, hcrtc);

	return ret;
}

int hisi_drm_crtc_irq_install(struct drm_device *dev, int irq,
			      unsigned long flags, void *data)
{
	int ret;

	if (!drm_core_check_feature(dev, DRIVER_HAVE_IRQ))
		return -EINVAL;

	if (irq == 0)
		return -EINVAL;

	/* Driver must have been initialized */
	if (!dev->dev_private)
		return -EINVAL;

	if (dev->irq_enabled)
		return -EBUSY;
	dev->irq_enabled = true;

	DRM_DEBUG("irq=%d\n", irq);

	ret = request_irq(irq, hisi_drm_crtc_irq_handler,
			  flags, dev->driver->name, data);

	if (ret < 0) {
		dev->irq_enabled = false;
		return ret;
	}

	return 0;
}

static void  hisi_drm_crtc_enable(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (hcrtc->enable)
		return;

	if (ops->enable)
		ops->enable(hcrtc);
	drm_crtc_vblank_on(crtc);

	hcrtc->enable = true;
}

static void hisi_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (!hcrtc->enable)
		return;

	drm_crtc_vblank_off(crtc);
	if (ops->disable)
		ops->disable(hcrtc);

	hcrtc->enable = false;
}

static void hisi_drm_crtc_mode_prepare(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->mode_prepare)
		ops->mode_prepare(hcrtc);
}

static bool hisi_drm_crtc_mode_fixup(struct drm_crtc *crtc,
			      const struct drm_display_mode *mode,
			      struct drm_display_mode *adj_mode)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;
	bool ret = true;

	if (ops->mode_fixup)
		ret = ops->mode_fixup(hcrtc, mode, adj_mode);

	return ret;
}

static void hisi_drm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->mode_set_nofb)
		ops->mode_set_nofb(hcrtc);
}

static void hisi_crtc_atomic_begin(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->atomic_begin)
		ops->atomic_begin(hcrtc);
}

static void hisi_crtc_atomic_flush(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->atomic_flush)
		ops->atomic_flush(hcrtc);
}

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.enable		= hisi_drm_crtc_enable,
	.disable	= hisi_drm_crtc_disable,
	.prepare	= hisi_drm_crtc_mode_prepare,
	.mode_fixup	= hisi_drm_crtc_mode_fixup,
	.mode_set_nofb	= hisi_drm_crtc_mode_set_nofb,
	.atomic_begin	= hisi_crtc_atomic_begin,
	.atomic_flush	= hisi_crtc_atomic_flush,
};

static void hisi_drm_crtc_destroy(struct drm_crtc *c)
{
	drm_crtc_cleanup(c);
}

static void hisi_crtc_atomic_reset(struct drm_crtc *crtc)
{
	struct hisi_crtc_state *state;

	if (crtc->state)
		kfree(to_hisi_crtc_state(crtc->state));

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state == NULL)
		return;

	/* set to default value */
	state->comp_type = COMPOSITION_UNKNOWN;

	crtc->state = &state->base;
	crtc->state->crtc = crtc;
}

static struct drm_crtc_state *hisi_crtc_atomic_duplicate_state(struct drm_crtc *crtc)
{
	struct hisi_crtc_state *state;
	struct hisi_crtc_state *copy;

	if (WARN_ON(!crtc->state))
		return NULL;

	state = to_hisi_crtc_state(crtc->state);
	copy = kmemdup(state, sizeof(*state), GFP_KERNEL);
	if (copy == NULL)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &copy->base);

	return &copy->base;
}

static void hisi_crtc_atomic_destroy_state(struct drm_crtc *crtc,
					  struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(crtc, state);
	kfree(to_hisi_crtc_state(state));
}

static int hisi_crtc_atomic_set_property(struct drm_crtc *crtc,
					 struct drm_crtc_state *state,
					 struct drm_property *property,
					 uint64_t val)
{
	struct hisi_drm_private *priv = crtc->dev->dev_private;
	struct hisi_crtc_state *hstate = to_hisi_crtc_state(state);

	DRM_DEBUG_DRIVER("\"%s\": val=%d\n", property->name, (int)val);

	if (property == priv->comp_type_prop)
		hstate->comp_type = val;
	else
		return -EINVAL;

	return 0;
}

static int hisi_crtc_atomic_get_property(struct drm_crtc *crtc,
					 const struct drm_crtc_state *state,
					 struct drm_property *property,
					 uint64_t *val)
{
	struct hisi_drm_private *priv = crtc->dev->dev_private;
	struct hisi_crtc_state *hstate = to_hisi_crtc_state(state);

	if (property == priv->comp_type_prop)
		*val =	hstate->comp_type;
	else
		return -EINVAL;

	return 0;
}


static const struct drm_crtc_funcs crtc_funcs = {
	.destroy	= hisi_drm_crtc_destroy,
	.set_config	= drm_atomic_helper_set_config,
	.page_flip	= drm_atomic_helper_page_flip,
	.reset		= hisi_crtc_atomic_reset,
	.set_property = drm_atomic_helper_crtc_set_property,
	.atomic_duplicate_state	= hisi_crtc_atomic_duplicate_state,
	.atomic_destroy_state	= hisi_crtc_atomic_destroy_state,
	.atomic_set_property = hisi_crtc_atomic_set_property,
	.atomic_get_property = hisi_crtc_atomic_get_property,
};

int hisi_drm_crtc_init(struct drm_device *dev,
		       struct hisi_crtc *hcrtc,
		       struct drm_plane *plane)
{
	struct hisi_crtc_ops *ops = hcrtc->ops;
	int ret;

	ret = drm_crtc_init_with_planes(dev, &hcrtc->base, plane,
					NULL, &crtc_funcs);
	if (ret) {
		DRM_ERROR("failed to init crtc.\n");
		return ret;
	}

	drm_crtc_helper_add(&hcrtc->base, &crtc_helper_funcs);

	if (ops->install_properties) {
		ret = ops->install_properties(dev, hcrtc);
		if (ret)
			return ret;
	}

	return 0;
}
