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
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic.h>

#include "hisi_drm_drv.h"
#include "hisi_drm_plane.h"
#include "hisi_drm_crtc.h"

#define to_hisi_plane(plane) \
		container_of(plane, struct hisi_plane, base)

static void hisi_plane_atomic_disable(struct drm_plane *plane,
			       struct drm_plane_state *old_state)
{
	struct hisi_plane *hplane = to_hisi_plane(plane);
	struct hisi_plane_funcs *ops = hplane->ops;

	DRM_DEBUG_DRIVER("enter.\n");

	if (!old_state->crtc)
		return;
	
	if (ops->atomic_disable)
		ops->atomic_disable(hplane, old_state);

	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_plane_atomic_update(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct hisi_plane *hplane = to_hisi_plane(plane);
	struct hisi_plane_funcs *ops = hplane->ops;
	struct drm_plane_state	*hstate	= plane->state;

	if (!hstate->crtc)
		return;
	
	if (ops->atomic_update)
		ops->atomic_update(hplane, old_state);
}

int hisi_plane_atomic_check(struct drm_plane *plane,
			    struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_crtc *crtc = state->crtc;
	struct drm_crtc_state *crtc_state;
	u32 src_x = state->src_x >> 16;
	u32 src_y = state->src_y >> 16;
	u32 src_w = state->src_w >> 16;
	u32 src_h = state->src_h >> 16;
	int crtc_x = state->crtc_x;
	int crtc_y = state->crtc_y;
	u32 crtc_w = state->crtc_w;
	u32 crtc_h = state->crtc_h;


	if (!crtc || !fb)
		return 0;

	if (state->rotation != BIT(DRM_ROTATE_0)) {
		DRM_ERROR("Rotation not support!!!\n");
		return -EINVAL;
	}

	crtc_state = drm_atomic_get_crtc_state(state->state, crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	if (src_w != crtc_w || src_h != crtc_h) {
		DRM_ERROR("Scale not support!!!\n");
		return -EINVAL;
	}

	if (src_x + src_w > fb->width ||
	    src_y + src_h > fb->height)
		return -EINVAL;

	if (crtc_x < 0 || crtc_y < 0)
		return -EINVAL;

	if (crtc_x + crtc_w > crtc_state->adjusted_mode.hdisplay ||
	    crtc_y + crtc_h > crtc_state->adjusted_mode.vdisplay)
		return -EINVAL;

	return 0;
}

void hisi_plane_cleanup_fb(struct drm_plane *plane,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *old_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
}

int hisi_plane_prepare_fb(struct drm_plane *p,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *new_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

static const struct drm_plane_helper_funcs hisi_plane_helper_funcs = {
	.prepare_fb = hisi_plane_prepare_fb,
	.cleanup_fb = hisi_plane_cleanup_fb,
	.atomic_check = hisi_plane_atomic_check,
	.atomic_update = hisi_plane_atomic_update,
	.atomic_disable = hisi_plane_atomic_disable,
};

void hisi_plane_destroy(struct drm_plane *plane)
{
	drm_plane_cleanup(plane);
}

static void hisi_plane_atomic_reset(struct drm_plane *plane)
{
	struct hisi_plane_state *state;

	if (plane->state && plane->state->fb)
		drm_framebuffer_unreference(plane->state->fb);

	if (plane->state)
		kfree(to_hisi_plane_state(plane->state));

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state == NULL)
		return;

	/* set to default value */
	state->zpos = plane->type == DRM_PLANE_TYPE_PRIMARY ? 0 :
		drm_plane_index(plane);
	state->base.rotation = BIT(DRM_ROTATE_0);
	state->alpha = 255;
	state->blend = ALPHA_BLENDING_NONE;

	plane->state = &state->base;
	plane->state->plane = plane;
}

static struct drm_plane_state *
hisi_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct hisi_plane_state *hstate;
	struct hisi_plane_state *copy;

	if (WARN_ON(!plane->state))
		return NULL;

	hstate = to_hisi_plane_state(plane->state);
	copy = kmemdup(hstate, sizeof(*hstate), GFP_KERNEL);
	if (copy == NULL)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);

	return &copy->base;
}

static void hisi_plane_atomic_destroy_state(struct drm_plane *plane,
					    struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(plane, state);
	kfree(to_hisi_plane_state(state));
}


static int hisi_plane_atomic_set_property(struct drm_plane *plane,
					  struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t val)
{
	struct hisi_drm_private *priv = plane->dev->dev_private;
	struct hisi_plane_state *hstate = to_hisi_plane_state(state);

	if (property == priv->zpos_prop)
		hstate->zpos = val;
	else if (property == priv->alpha_prop)
		hstate->alpha = val;
	else if (property == priv->blend_prop)
		hstate->blend = val;
	else
		return -EINVAL;

	return 0;
}

static int hisi_plane_atomic_get_property(struct drm_plane *plane,
					  const struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t *val)
{
	struct hisi_drm_private *priv = plane->dev->dev_private;
	const struct hisi_plane_state *hstate = to_hisi_plane_state(state);

	if (property == priv->zpos_prop)
		*val = hstate->zpos;
	else if (property == priv->alpha_prop)
		*val = hstate->alpha;
	else if (property == priv->blend_prop)
		*val = hstate->blend;
	else
		return -EINVAL;

	return 0;
}

static struct drm_plane_funcs hisi_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.set_property = drm_atomic_helper_plane_set_property,
	.destroy = hisi_plane_destroy,
	.reset = hisi_plane_atomic_reset,
	.atomic_duplicate_state = hisi_plane_atomic_duplicate_state,
	.atomic_destroy_state = hisi_plane_atomic_destroy_state,
	.atomic_set_property = hisi_plane_atomic_set_property,
	.atomic_get_property = hisi_plane_atomic_get_property,
};

int hisi_drm_plane_init(struct drm_device *dev,
			struct hisi_plane *hplane,
			enum drm_plane_type type)
{
	struct hisi_plane_funcs *ops = hplane->ops;
	const u32 *fmts;
	u32 fmts_cnt;
	int ret = 0;

	/* get  properties */
	fmts_cnt = ops->get_properties(hplane->ch, &fmts);
	if (ret)
		return ret;

	ret = drm_universal_plane_init(dev, &hplane->base, 1,
				&hisi_plane_funcs, fmts, fmts_cnt, type);
	if (ret) {
		DRM_ERROR("fail to init plane, ch=%d\n", hplane->ch);
		return ret;
	}

	drm_plane_helper_add(&hplane->base, &hisi_plane_helper_funcs);

	/* install overlay plane properties */
	if (type == DRM_PLANE_TYPE_OVERLAY && ops->install_properties) {
		ret = ops->install_properties(dev, hplane);
		if (ret)
			return ret;
	}

	return 0;
}
