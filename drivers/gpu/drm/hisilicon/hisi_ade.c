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

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <video/display_timing.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_drm_fb.h"
#include "hisi_drm_fbdev.h"
#include "hisi_drm_drv.h"
#include "hisi_drm_plane.h"
#include "hisi_drm_crtc.h"
#include "hisi_ade_reg.h"

#define FORCE_PIXEL_CLOCK_SAME_OR_HIGHER 0
#define PRIMARY_CH	(ADE_CH1)

#define to_ade_crtc(hcrtc)	container_of(hcrtc, struct ade_crtc, base)

struct ade_crtc {
	struct hisi_crtc base;
	struct drm_display_mode *dmode;
	u32 ch_mask;
	u64 use_mask;
};

struct ade_hardware_context {
	void __iomem  *base;
	void __iomem  *media_base;

	int irq;
	u32 ade_core_rate;
	u32 media_noc_rate;

	struct clk *ade_core_clk;
	struct clk *media_noc_clk;
	struct clk *ade_pix_clk;
	bool power_on;
};

struct hisi_ade {
	struct ade_crtc acrtc;
	struct hisi_plane hplane[ADE_CH_NUM];
	struct ade_hardware_context ctx;
};

/* ade-format info: */
struct ade_format {
	u32 pixel_format;
	enum ADE_FORMAT ade_format;
};

static const struct ade_format ade_formats[] = {
	/* 16bpp RGB: */
	{ DRM_FORMAT_RGB565, ADE_RGB_565 },
	{ DRM_FORMAT_BGR565, ADE_BGR_565 },
	/* 24bpp RGB: */
	{ DRM_FORMAT_RGB888, ADE_RGB_888 },
	{ DRM_FORMAT_BGR888, ADE_BGR_888 },
	/* 32bpp [A]RGB: */
	{ DRM_FORMAT_XRGB8888, ADE_XRGB_8888 },
	{ DRM_FORMAT_XBGR8888, ADE_XBGR_8888 },
	{ DRM_FORMAT_RGBA8888, ADE_RGBA_8888 },
	{ DRM_FORMAT_BGRA8888, ADE_BGRA_8888 },
	{ DRM_FORMAT_ARGB8888, ADE_ARGB_8888 },
	{ DRM_FORMAT_ABGR8888, ADE_ABGR_8888 },
	/* packed YCbCr */
	{ DRM_FORMAT_YUYV, ADE_YUYV },
	{ DRM_FORMAT_YVYU, ADE_YVYU },
	{ DRM_FORMAT_UYVY, ADE_UYVY },
	{ DRM_FORMAT_VYUY, ADE_VYUY },
	/* 2 plane YCbCr */
	{ DRM_FORMAT_NV12, ADE_NV12 },
	{ DRM_FORMAT_NV21, ADE_NV21 },
	/* 3 plane YCbCr */
	{ DRM_FORMAT_YUV444, ADE_YUV444 },
};

static const uint32_t channel_formats1[] = {
	/* channel 1,2,3,4 */
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565, DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888, DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888, DRM_FORMAT_BGRA8888, DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888
};

static const uint32_t channel_formats2[] = {
	/* channel 5,6 */
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565, DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888, DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888, DRM_FORMAT_BGRA8888, DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888, DRM_FORMAT_YUYV, DRM_FORMAT_YVYU, DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY, DRM_FORMAT_NV12, DRM_FORMAT_NV21, DRM_FORMAT_YUV444
};

static const uint32_t channel_formats3[] = {
	/* disp channel 7 */
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565, DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888, DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888, DRM_FORMAT_BGRA8888, DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888, DRM_FORMAT_YUYV, DRM_FORMAT_YVYU, DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY, DRM_FORMAT_YUV444
};

static void ade_display_commit(struct ade_crtc *acrtc);

u32 ade_get_channel_formats(u8 ch, const u32 **formats)
{
	switch(ch)
	{
	case ADE_CH1:
	case ADE_CH2:
	case ADE_CH3:
	case ADE_CH4:
		*formats = channel_formats1;
		return ARRAY_SIZE(channel_formats1);
	case ADE_CH5:
	case ADE_CH6:
		*formats = channel_formats2;
		return ARRAY_SIZE(channel_formats2);
	case ADE_DISP:
		*formats = channel_formats3;
		return ARRAY_SIZE(channel_formats3);
	default:
		DRM_ERROR("no this channel %d\n", ch);
		*formats = NULL;
		return 0;
	}
}

/* convert from fourcc format to ade format */
static u32 ade_get_format(u32 pixel_format)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ade_formats); i++)
		if (ade_formats[i].pixel_format == pixel_format)
			return ade_formats[i].ade_format;

	/* not found */
	DRM_ERROR("Not found pixel format!!fourcc_format= %d\n", pixel_format);
	return ADE_FORMAT_NOT_SUPPORT;
}

static bool ade_is_need_csc(u32 fmt)
{
	switch (fmt) {
        case ADE_YUYV:
        case ADE_YVYU:
        case ADE_UYVY:
        case ADE_VYUY:
        case ADE_YUV444:
        case ADE_NV12:
        case ADE_NV21:
            return true;
        default:
            return false;
	}
}

static void ade_init(struct ade_hardware_context *ctx)
{
	void __iomem *base = ctx->base;

	/* enable clk gate */
	set_TOP_CTL_clk_gate_en(base, 1);
	/* clear overlay */
	writel(0, base + ADE_OVLY1_TRANS_CFG);
	writel(0, base + ADE_OVLY_CTL);
	writel(0, base + ADE_OVLYX_CTL(ADE_OVLY2));
	/* clear reset and reload regs */
	writel(0, base + ADE_SOFT_RST_SEL0);
	writel(0, base + ADE_SOFT_RST_SEL1);
	writel(0xFFFFFFFF, base + ADE_RELOAD_DIS0);
	writel(0xFFFFFFFF, base + ADE_RELOAD_DIS1);
	/* for video set to 1, means that ade registers
	 * became effective at frame end */
	set_TOP_CTL_frm_end_start(base, 1);
}

static void ade_ldi_set_mode(struct ade_hardware_context *ctx, struct drm_display_mode *mode)
{
	void __iomem *base = ctx->base;
	u32 hfp, hbp, hsw, vfp, vbp, vsw;
	u32 plr_flags;

	plr_flags = (mode->flags & DRM_MODE_FLAG_NVSYNC)
			? HISI_LDI_FLAG_NVSYNC : 0;
	plr_flags |= (mode->flags & DRM_MODE_FLAG_NHSYNC)
			? HISI_LDI_FLAG_NHSYNC : 0;
	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	hsw = mode->hsync_end - mode->hsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;
	vsw = mode->vsync_end - mode->vsync_start;
	if (vsw > 15) {
		pr_err("%s: vsw exceeded 15\n", __func__);
		vsw = 15;
	}

	writel((hbp << 20) | (hfp << 0), base + LDI_HRZ_CTRL0);
	/* p3-73 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(hsw - 1, base + LDI_HRZ_CTRL1);
	writel((vbp << 20) | (vfp << 0), base + LDI_VRT_CTRL0);
	/* p3-74 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(vsw - 1, base + LDI_VRT_CTRL1);

	/* p3-75 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(((mode->vdisplay - 1) << 20) | ((mode->hdisplay - 1) << 0),
	       base + LDI_DSP_SIZE);
	writel(plr_flags, base + LDI_PLR_CTRL);

	/*
	 * other parameters setting
	 */
	writel(BIT(0), base + LDI_WORK_MODE);
	writel((0x3c << 6) | (ADE_OUT_RGB_888 << 3) | BIT(2) | BIT(0),
	       base + LDI_CTRL);
	set_reg(base + LDI_DE_SPACE_LOW, 0x1, 1, 1);
}

static int ade_power_up(struct ade_hardware_context *ctx)
{
	void __iomem *media_base = ctx->media_base;
	int ret;

	ret = clk_set_rate(ctx->ade_core_clk, ctx->ade_core_rate);
	if (ret) {
		DRM_ERROR("clk_set_rate ade_core_rate error\n");
		return ret;
	}
	ret = clk_set_rate(ctx->media_noc_clk, ctx->media_noc_rate);
	if (ret) {
		DRM_ERROR("media_noc_clk media_noc_rate error\n");
		return ret;
	}
	ret = clk_prepare_enable(ctx->media_noc_clk);
	if (ret) {
		DRM_ERROR("fail to clk_prepare_enable media_noc_clk\n");
		return ret;
	}

	writel(0x20, media_base + SC_MEDIA_RSTDIS);

	ret = clk_prepare_enable(ctx->ade_core_clk);
	if (ret) {
		DRM_ERROR("fail to clk_prepare_enable ade_core_clk\n");
		return ret;
	}

	ade_init(ctx);
	ctx->power_on = true;
	return 0;
}

static void ade_power_down(struct ade_hardware_context *ctx)
{
	void __iomem *base = ctx->base;
	void __iomem *media_base = ctx->media_base;

	set_LDI_CTRL_ldi_en(base, ADE_DISABLE);
	/* dsi pixel off */
	set_reg(base + LDI_HDMI_DSI_GT, 0x1, 1, 0);

	clk_disable_unprepare(ctx->ade_core_clk);
	writel(0x20, media_base + SC_MEDIA_RSTEN);
	clk_disable_unprepare(ctx->media_noc_clk);
	ctx->power_on = false;
}

static void ade_crtc_enable(struct hisi_crtc *hcrtc)
{
	struct ade_crtc *acrtc = to_ade_crtc(hcrtc);
	struct ade_hardware_context *ctx = hcrtc->ctx;

	int ret;

	if (!ctx->power_on) {
		ret = ade_power_up(ctx);
		if (ret) {
			DRM_ERROR("failed to initialize ade clk\n");
			return ;
		}
	}

	ade_display_commit(acrtc);

	DRM_DEBUG_DRIVER("exit success.\n");
}

static void ade_crtc_disable(struct hisi_crtc *hcrtc)
{
	struct ade_crtc *acrtc = to_ade_crtc(hcrtc);
	struct ade_hardware_context *ctx = hcrtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");

	ade_power_down(ctx);

	acrtc->ch_mask = 0;
	acrtc->use_mask = 0;
	DRM_DEBUG_DRIVER("exit success.\n");
}

bool ade_crtc_mode_fixup(struct hisi_crtc *hcrtc,
			 const struct drm_display_mode *mode,
			 struct drm_display_mode *adj_mode)
{
	struct ade_hardware_context *ctx = hcrtc->ctx;
	u32 clock_kHz = mode->clock;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");

	if (!ctx->power_on)
		if (ade_power_up(ctx))
			DRM_ERROR("%s: failed to power up ade\n", __func__);

	do {
		ret = clk_set_rate(ctx->ade_pix_clk, clock_kHz * 1000);
		if (ret) {
			DRM_ERROR("set ade_pixel_clk_rate fail\n");
			return false;
		}
		adj_mode->clock = clk_get_rate(ctx->ade_pix_clk) / 1000;
#if FORCE_PIXEL_CLOCK_SAME_OR_HIGHER
		if (adj_mode->clock >= clock_kHz)
#endif
		/* This avoids a bad 720p DSI clock with 1.2GHz DPI PLL */
		if (adj_mode->clock != 72000)
			break;

		clock_kHz += 10;
	} while (1);

	pr_info("%s: pixel clock: req %dkHz -> actual: %dkHz\n",
		__func__, mode->clock, adj_mode->clock);

	DRM_DEBUG_DRIVER("mode_fixup  exit successfully.\n");
	return true;
}

void ade_crtc_mode_set_nofb(struct hisi_crtc *hcrtc)
{
	struct ade_crtc *acrtc = to_ade_crtc(hcrtc);
	struct ade_hardware_context *ctx = hcrtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	acrtc->dmode = &hcrtc->base.state->mode;
	ade_ldi_set_mode(ctx, &hcrtc->base.state->mode);
	DRM_DEBUG_DRIVER("exit success.\n");
}

void ade_crtc_atomic_begin(struct hisi_crtc *hcrtc)
{
	struct ade_hardware_context *ctx = hcrtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!ctx->power_on)
		(void) ade_power_up(ctx);
	DRM_DEBUG_DRIVER("exit success.\n");
}

/*
 * set modules' reset mode: by software or hardware
 * set modules' reload enable/disable
 * */
static void ade_set_reset_and_reload(struct ade_crtc *acrtc)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	u32 mask0 = (u32)acrtc->use_mask;
	u32 mask1 = (u32)(acrtc->use_mask >> 32);

	DRM_DEBUG_DRIVER("mask=0x%llX, mask0=0x%X, mask1=0x%X\n",
			acrtc->use_mask, mask0, mask1);

	writel(mask0, base + ADE_SOFT_RST_SEL0);
	writel(mask1, base + ADE_SOFT_RST_SEL1);
	writel(~mask0, base + ADE_RELOAD_DIS0);
	writel(~mask1, base + ADE_RELOAD_DIS1);
}

/*
 * commit to ldi to display
 */
static void ade_display_commit(struct ade_crtc *acrtc)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	u32 out_w = acrtc->dmode->hdisplay;
	u32 out_h = acrtc->dmode->vdisplay;

	/* TODO: set rotator after overlay */

	/* TODO: set scale after overlay */

	/* display source setting */
	writel(TOP_DISP_SRC_OVLY2, base + ADE_DISP_SRC_CFG);
	
	/* ctran6 setting */
	writel(1, base + ADE_CTRAN_DIS(ADE_CTRAN6));
	writel(out_w * out_h - 1, base + ADE_CTRAN_IMAGE_SIZE(ADE_CTRAN6));
	acrtc->use_mask |= BIT(ADE_CTRAN_BIT_OFST + ADE_CTRAN6);

	/* set reset mode:soft or hw, and reload modules */
	ade_set_reset_and_reload(acrtc);

	DRM_INFO("ADE GO: %dx%d \n", out_w, out_h);
	/* enable ade */
	wmb();
	writel(ADE_ENABLE, base + ADE_EN);
	wmb();
	set_LDI_CTRL_ldi_en(base, ADE_ENABLE);
	/* dsi pixel on */
	set_reg(base + LDI_HDMI_DSI_GT, 0x0, 1, 0);
}

void ade_crtc_atomic_flush(struct hisi_crtc *hcrtc)

{
	struct ade_crtc *acrtc = to_ade_crtc(hcrtc);
	struct ade_hardware_context *ctx = hcrtc->ctx;
	void __iomem *base = ctx->base;

	DRM_DEBUG_DRIVER("enter.\n");
	/* commit to  display: LDI input setting */
	if (hcrtc->enable) {
		/* set reset and reload */
		ade_set_reset_and_reload(acrtc);
		/* flush ade regitsters */
		wmb();
		writel(ADE_ENABLE, base + ADE_EN);
	}
	DRM_DEBUG_DRIVER("exit success.\n");
}

void ade_crtc_mode_prepare(struct hisi_crtc *hcrtc)
{
	struct ade_hardware_context *ctx = hcrtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!ctx->power_on)
		(void) ade_power_up(ctx);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static int ade_dts_parse(struct platform_device *pdev,
			      struct ade_hardware_context *ctx)
{
	struct resource *res;
	struct device *dev;
	struct device_node *np;
	int ret;

	dev = &pdev->dev;
	np  = dev->of_node;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ade_base");
	ctx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->base)) {
		DRM_ERROR("failed to remap ade io base\n");
		return  PTR_ERR(ctx->base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "media_base");
	ctx->media_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->media_base)) {
		DRM_ERROR("failed to remap media io base\n");
		return PTR_ERR(ctx->media_base);
	}

	ctx->irq = platform_get_irq(pdev, 0);
	if (ctx->irq < 0) {
		DRM_ERROR("failed to parse the irq\n");
		return -ENODEV;
	}

	ctx->ade_core_clk = devm_clk_get(&pdev->dev, "clk_ade_core");
	if (ctx->ade_core_clk == NULL) {
		DRM_ERROR("failed to parse the ADE_CORE\n");
		return -ENODEV;
	}
	ctx->media_noc_clk = devm_clk_get(&pdev->dev,
					"aclk_codec_jpeg_src");
	if (ctx->media_noc_clk == NULL) {
		DRM_ERROR("failed to parse the CODEC_JPEG\n");
	    return -ENODEV;
	}
	ctx->ade_pix_clk = devm_clk_get(&pdev->dev, "clk_ade_pix");
	if (ctx->ade_pix_clk == NULL) {
		DRM_ERROR("failed to parse the ADE_PIX_SRC\n");
	    return -ENODEV;
	}

	ret = of_property_read_u32(np, "ade_core_clk_rate",
				    &ctx->ade_core_rate);
	if (ret) {
		DRM_ERROR("failed to parse the ade_core_clk_rate\n");
	    return -ENODEV;
	}
	ret = of_property_read_u32(np, "media_noc_clk_rate",
				    &ctx->media_noc_rate);
	if (ret) {
		DRM_ERROR("failed to parse the media_noc_clk_rate\n");
		return -ENODEV;
	}

	return 0;
}

int ade_enable_vblank(struct hisi_crtc *hcrtc)
{
	struct ade_hardware_context *ctx = hcrtc->ctx;
	void __iomem *base = ctx->base;
	u32 intr_en;

	DRM_INFO("enable_vblank enter.\n");
	if (!ctx->power_on)
		(void) ade_power_up(ctx);

	intr_en = readl(base + LDI_INT_EN);
	intr_en |= LDI_ISR_FRAME_END_INT;
	writel(intr_en, base + LDI_INT_EN);

	return 0;
}

void ade_disable_vblank(struct hisi_crtc *hcrtc)
{
	struct ade_hardware_context *ctx = hcrtc->ctx;
	void __iomem *base = ctx->base;
	u32 intr_en;

	DRM_INFO("disable_vblank enter.\n");
	if (!ctx->power_on) {
		DRM_ERROR("power is down! vblank disable fail\n");
		return ;
	}
	intr_en = readl(base + LDI_INT_EN);
	intr_en &= ~LDI_ISR_FRAME_END_INT;
	writel(intr_en, base + LDI_INT_EN);
}

irqreturn_t ade_irq_handler(int irq, struct hisi_crtc *hcrtc)
{
	struct ade_hardware_context *ctx = hcrtc->ctx;
	struct drm_crtc *crtc = &hcrtc->base;
	struct drm_device *dev = crtc->dev;
	void __iomem *base = ctx->base;
	u32 status;

	status = readl(base + LDI_MSK_INT);
	/* DRM_INFO("LDI IRQ: status=0x%X\n",status); */

	/* vblank irq */
	if (status & LDI_ISR_FRAME_END_INT) {
		writel(LDI_ISR_FRAME_END_INT, base + LDI_INT_CLR);
		drm_handle_vblank(dev, drm_crtc_index(crtc));
	}

	return IRQ_HANDLED;
}

static void ade_rdma_set(struct ade_crtc *acrtc, struct drm_framebuffer *fb, u32 ch,
			 u32 y, u32 in_h, u32 fmt, u32 rotation)
{
	u32 reg_ctrl, reg_addr, reg_size, reg_stride, reg_space, reg_en;
	struct drm_gem_cma_object *obj = hisi_drm_fb_get_gem_obj(fb, 0);
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	u32 stride = fb->pitches[0];
	u32 addr = (u32)obj->paddr + y * stride;

	DRM_DEBUG_DRIVER("rdma%d: (y=%d, height=%d), stride=%d, paddr=0x%x,"
			 "addr=0x%x, fb:%dx%d, pixel_format=%d(%s)\n",
			 ch+1, y, in_h, stride, (u32)obj->paddr,
			 addr, fb->width, fb->height,
			 fmt, drm_get_format_name(fb->pixel_format));

	/* get reg offset */
	switch(ch) {
	case ADE_DISP:
		reg_ctrl = RD_CH_DISP_CTRL;
		reg_addr = RD_CH_DISP_ADDR;
		reg_size = RD_CH_DISP_SIZE;
		reg_stride = RD_CH_DISP_STRIDE;
		reg_space = RD_CH_DISP_SPACE;
		reg_en = RD_CH_DISP_EN;
		break;
	default:
		reg_ctrl = RD_CH_CTRL(ch);
		reg_addr = RD_CH_ADDR(ch);
		reg_size = RD_CH_SIZE(ch);
		reg_stride = RD_CH_STRIDE(ch);
		reg_space = RD_CH_SPACE(ch);
		reg_en = RD_CH_EN(ch);
		break;
	}

	/*
	 * TODO: set rotation
	 */
	writel((fmt << 16) & 0x1f0000, base + reg_ctrl);
	writel(addr, base + reg_addr);
	writel((in_h << 16) | stride, base + reg_size);
	writel(stride, base + reg_stride);
	writel(in_h * stride, base + reg_space);
	writel(1, base + reg_en);
	
	acrtc->use_mask |= BIT(ADE_CH_RDMA_BIT_OFST + ch);
}

static void ade_rdma_disable(struct ade_crtc *acrtc, u32 ch)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	u32 reg_en;

	/* get reg offset */
	switch(ch) {
	case ADE_DISP:
		reg_en = RD_CH_DISP_EN;
		break;
	default:
		reg_en = RD_CH_EN(ch);
		break;
	}

	writel(0, base + reg_en);
	acrtc->use_mask &= ~BIT(ADE_CH_RDMA_BIT_OFST + ch);
}

static void ade_clip_set(struct ade_crtc *acrtc, u32 ch, u32 fb_w, u32 x,
			 u32 in_w, u32 in_h)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	u32 disable_val;
	u32 clip_left;
	u32 clip_right;

	/* ADE_DISP channel has no clip module */
	if (ch == ADE_DISP)
		return;

	/*
	 * clip width, no need to clip height
	 */
	if (fb_w == in_w) { /* bypass */
		disable_val = 1;
		clip_left = 0;
		clip_right = 0;
	} else {
		disable_val = 0;
		clip_left = x;
		clip_right = fb_w - (x + in_w) - 1;
	}

	DRM_DEBUG_DRIVER("clip%d: reg_disable=0x%X, reg_size0=0x%X, reg_size1=0x%X\n",
			ch+1, ADE_CLIP_DISABLE(ch), ADE_CLIP_SIZE0(ch),
			ADE_CLIP_SIZE1(ch));

	DRM_DEBUG_DRIVER("clip%d: clip_left=%d, clip_right=%d\n",
			ch+1, clip_left, clip_right);

	writel(disable_val, base + ADE_CLIP_DISABLE(ch));
	writel((fb_w - 1) << 16 | (in_h - 1), base + ADE_CLIP_SIZE0(ch));
	writel(clip_left << 16 | clip_right, base + ADE_CLIP_SIZE1(ch));

	acrtc->use_mask |= BIT(ADE_CLIP_BIT_OFST + ch);
}

static void ade_clip_disable(struct ade_crtc *acrtc, u32 ch)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;

	if (ch == ADE_DISP)
		return;

	writel(1, base + ADE_CLIP_DISABLE(ch));
	acrtc->use_mask &= ~BIT(ADE_CLIP_BIT_OFST + ch);
}

static void ade_scale_set(struct ade_crtc *acrtc, u32 ch, u32 in_w, u32 in_h,
			  u32 *out_w, u32 *out_h)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	bool need_scale = false;
	u32 o_w, o_h;
	u32 ctrl_val;
	u8 x;

	switch (ch) {
	case ADE_CH5:
		x = ADE_SCL3;
		break;
	case ADE_CH6:
		x = ADE_SCL1;
		break;
	default: /* channel 1,2,3,4,disp has no scale capability */
		return;
	}

	if (!need_scale) {/* bypass */
		ctrl_val = 0x400;
		o_w = in_w;
		o_h = in_h;
	} else {/* TODO: scale setting
		ctrl_val = 0x400;
		o_w = ;
		o_h = ; */
	}

	DRM_DEBUG_DRIVER("channel%d, scl%d: reg_ctrl=0x%X, reg_ires=0x%X, reg_ores=0x%X, reg_start=0x%X\n",
			 ch+1, x+1, ADE_SCL_CTRL(x), ADE_SCL_IRES(x), ADE_SCL_ORES(x),
			 ADE_SCL_START(x));

	writel(ctrl_val, base + ADE_SCL_CTRL(x));
	writel((in_h - 1) << 16 | (in_w - 1), base + ADE_SCL_IRES(x));
	writel((o_h - 1) << 16 | (o_w - 1), base + ADE_SCL_ORES(x));
	writel(1, base + ADE_SCL_START(x));

	*out_w = o_w;
	*out_h = o_h;

	acrtc->use_mask |= BIT(ADE_SCL_BIT_OFST + x);
}

static void ade_scale_disable(struct ade_crtc *acrtc, u32 ch)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	u8 x;

	switch (ch) {
	case ADE_CH5:
		x = ADE_SCL3;
		break;
	case ADE_CH6:
		x = ADE_SCL1;
		break;
	default: /* channel 1,2,3,4,disp has no scale capability */
		return;
	}

	writel(0, base + ADE_SCL_START(x));
	acrtc->use_mask &= ~BIT(ADE_SCL_BIT_OFST + x);
}

/*
 * corlor space converting
 * */
static void ade_ctran_set(struct ade_crtc *acrtc, u32 ch, u32 in_w, u32 in_h,
			  u32 fmt)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	bool need_ctran = ade_is_need_csc(fmt);
	u8 x;

	switch (ch) {
	case ADE_DISP:
		x = ADE_CTRAN5;
		break;
	case ADE_CH5:
		x = ADE_CTRAN1;
		break;
	case ADE_CH6:
		x = ADE_CTRAN2;
		break;
	default: /* channel 1,2,3,4 has no csc capability */
		return;
	}

	if (!need_ctran) {/* bypass */
		writel(1, base + ADE_CTRAN_DIS(x));
		writel(in_w * in_h - 1, base + ADE_CTRAN_IMAGE_SIZE(x));
	} else {/* TODO */
	}

	acrtc->use_mask |= BIT(ADE_CTRAN_BIT_OFST + x);
}

static void ade_ctran_disable(struct ade_crtc *acrtc, u32 ch)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	u8 x;

	switch (ch) {
	case ADE_DISP:
		x = ADE_CTRAN5;
		break;
	case ADE_CH5:
		x = ADE_CTRAN1;
		break;
	case ADE_CH6:
		x = ADE_CTRAN2;
		break;
	default: /* channel 1,2,3,4 has no csc capability */
		return;
	}
	

	writel(1, base + ADE_CTRAN_DIS(x));
	
	acrtc->use_mask &= ~BIT(ADE_CTRAN_BIT_OFST + x);
}

static bool has_Alpha_channel(int format)
{
	switch (format) {
	case ADE_ARGB_8888:
	case ADE_ABGR_8888:
	case ADE_RGBA_8888:
	case ADE_BGRA_8888:
		return true;
	default:
		return false;
	}
}

static void ade_get_blending_params(u32 blend, u32 fmt, u8 glb_alpha,
				    u8 *alp_mode, u8 *alp_sel, u8 *under_alp_sel)
{
	bool has_alpha = has_Alpha_channel(fmt);


	/*
	 * get alp_mode
	 */
	if (has_alpha && glb_alpha < 0xFF)
		*alp_mode = ADE_ALP_PIXEL_AND_GLB;
	else if (has_alpha)
		*alp_mode = ADE_ALP_PIXEL;
	else 
		*alp_mode = ADE_ALP_GLOBAL;

	/*
	 * get alp sel
	 */
	*alp_sel = ADE_ALP_MUL_COEFF_3; /* 1 */
	*under_alp_sel = ADE_ALP_MUL_COEFF_1; /* 1 - alpha */
	switch(blend) {
	case ALPHA_BLENDING_PREMULT:
		break;
	case ALPHA_BLENDING_COVERAGE:
                *alp_sel = ADE_ALP_MUL_COEFF_0; /* alpha */
		break;
	case ALPHA_BLENDING_NONE:
		*under_alp_sel = ADE_ALP_MUL_COEFF_2; /* 0 */
		break;
	default:
		DRM_ERROR("unsupport blending=0x%X\n", blend);
		break;
	}
}

static void ade_overlay_set(struct ade_crtc *acrtc, u8 ch, u8 zpos, u32 x0,
			    u32 y0, u32 in_w, u32 in_h, u32 blend,
			    u8 glb_alpha, u32 fmt)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	struct hisi_crtc_state *state = to_hisi_crtc_state(acrtc->base.base.state);
	u8 comp_type = state->comp_type;
	u8 ovly_ch = zpos;
	u8 x = ADE_OVLY2;
	u32 x1 = x0 + in_w - 1;
	u32 y1 = y0 + in_h - 1;
	u32 val;
	u8 alp_sel;
	u8 under_alp_sel;
	u8 alp_mode;

	ade_get_blending_params(blend, fmt, glb_alpha, &alp_mode, &alp_sel,
				    &under_alp_sel);

	/*
	 * when all the HWC layers are composed by HWC, target layer(aka primary
	 * plane) should be ignored. So make primary plane transparent.
	 */
#if 1
	if (ch == PRIMARY_CH) {
		if (comp_type == COMPOSITION_HWC) {
			alp_sel = ADE_ALP_MUL_COEFF_2; /* 0 */
		}
		else if (comp_type == COMPOSITION_GLES) {
			under_alp_sel = ADE_ALP_MUL_COEFF_2; /* 0 */
		}
	}
#endif

	DRM_DEBUG_DRIVER("channel%d--> ovly_ch=%d: alp_mode=%d, alp_sel=%d, under_alp_sel=%d, composition_type=%d\n",
			ch+1, ovly_ch+1,
			alp_mode,
			alp_sel,
			under_alp_sel,
			comp_type);

	/* overlay routing setting */
	writel(x0 << 16 | y0, base + ADE_OVLY_CH_XY0(ovly_ch));
	writel(x1 << 16 | y1, base + ADE_OVLY_CH_XY1(ovly_ch));
	val = (ch + 1) << ADE_OVLY_CH_SEL_OFST | BIT(ADE_OVLY_CH_EN_OFST) |
		alp_sel << ADE_OVLY_CH_ALP_SEL_OFST |
		under_alp_sel << ADE_OVLY_CH_UNDER_ALP_SEL_OFST |
		glb_alpha << ADE_OVLY_CH_ALP_GBL_OFST |
		alp_mode << ADE_OVLY_CH_ALP_MODE_OFST;
	DRM_DEBUG_DRIVER("ch%d_ctl=0x%X\n", ovly_ch+1, val);
	writel(val, base + ADE_OVLY_CH_CTL(ovly_ch));
	val= (x + 1) << (ovly_ch * 4) | readl(base + ADE_OVLY_CTL);
	DRM_DEBUG_DRIVER("ovly_ctl=0x%X\n", val);
	writel(val, base + ADE_OVLY_CTL);

	if (ch == ADE_DISP)
		writel(1, base + ADE_CTRAN5_TRANS_CFG);

	/* when primary is enable, indicate that it's ready to output. */
	if (ch == PRIMARY_CH) {
		val = (in_w - 1 ) << 16 | (in_h - 1);
		writel(val, base + ADE_OVLY_OUTPUT_SIZE(x));
		writel(1, base + ADE_OVLYX_CTL(x));
		acrtc->use_mask |= BIT(ADE_OVLY_BIT_OFST + x);
	}
}

static void ade_overlay_disable(struct ade_crtc *acrtc, u32 ch, u8 zpos)
{
	struct ade_hardware_context *ctx = acrtc->base.ctx;
	void __iomem *base = ctx->base;
	u8 ovly_ch = zpos;
	u32 val;

	val = ~BIT(6) & readl(base + ADE_OVLY_CH_CTL(ovly_ch));
	DRM_DEBUG_DRIVER("ch%d_ctl=0x%X\n", ovly_ch+1, val);
	writel(val, base + ADE_OVLY_CH_CTL(ovly_ch));
	val = ~(0x3 << (ovly_ch * 4)) & readl(base + ADE_OVLY_CTL);

	DRM_DEBUG_DRIVER("ovly_ctl=0x%X\n", val);
	writel(val, base + ADE_OVLY_CTL);

	if (ch == ADE_DISP)
		writel(0, base + ADE_CTRAN5_TRANS_CFG);

}

/*
 * Typicaly, a channel looks like: DMA-->clip-->scale-->ctrans-->overlay
 */
static void ade_update_channel(struct hisi_plane *hplane, struct ade_crtc *acrtc,
			  struct drm_framebuffer *fb, int crtc_x, int crtc_y,
			  unsigned int crtc_w, unsigned int crtc_h,
			  uint32_t src_x, uint32_t src_y,
			  uint32_t src_w, uint32_t src_h)
{
	struct drm_plane_state	*state = hplane->base.state;
	struct hisi_plane_state *hstate = to_hisi_plane_state(state);
	u8 ch = hplane->ch;
	u8 zpos = hstate->zpos;
	u8 glb_alpha = hstate->alpha;
	u32 blend = hstate->blend;
	u32 rotation = state->rotation;
	u32 fmt = ade_get_format(fb->pixel_format);
	u32 in_w;
	u32 in_h;
	u32 out_w;
	u32 out_h;

	DRM_DEBUG_DRIVER("channel%d: src:(%d, %d)-%dx%d, crtc:(%d, %d)-%dx%d, "
			"zpos=%d, alpha=%d, blend=0x%x, rotation=%d\n",
			ch+1, src_x, src_y, src_w, src_h,
			crtc_x, crtc_y, crtc_w, crtc_h,
			zpos, glb_alpha, blend, rotation);

	/* 1) DMA setting */
	in_w = src_w;
	in_h = src_h;
	ade_rdma_set(acrtc, fb, ch, src_y, in_h, fmt, rotation);

	/* 2) clip setting */
	ade_clip_set(acrtc, ch, fb->width, src_x, in_w, in_h);

	/* 3) scale setting */
	out_w = crtc_w;
	out_h = crtc_h;
	ade_scale_set(acrtc, ch, in_w, in_h, &out_w, &out_h);

	/* 4) ctran/csc setting */
	in_w = out_w;
	in_h = out_h;
	ade_ctran_set(acrtc, ch, in_w, in_h, fmt);

	/* 5) overlay routing setting */
	ade_overlay_set(acrtc, ch, zpos, crtc_x, crtc_y, in_w, in_h, blend,
			glb_alpha, fmt);

	acrtc->ch_mask |= BIT(ch);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void ade_disable_channel(struct hisi_plane *hplane, struct ade_crtc *acrtc)
{
	struct drm_plane_state  *state = hplane->base.state;
	struct hisi_plane_state *hstate = to_hisi_plane_state(state);
	u32 ch = hplane->ch;
	u8 zpos = hstate->zpos;

	DRM_DEBUG_DRIVER("disable channel%d\n", ch+1);
	/* reset state */
	hstate->zpos = hplane->base.type == DRM_PLANE_TYPE_PRIMARY ? 0 :
		drm_plane_index(&hplane->base);
	state->rotation = BIT(DRM_ROTATE_0);
	hstate->alpha = 255;
	hstate->blend = ALPHA_BLENDING_NONE;

	/*
	 * when primary is disable, power is down
	 * so no need to disable this channel.
	 */
	if (ch == PRIMARY_CH)
		return;

	/* disable read DMA */
	ade_rdma_disable(acrtc, ch);

	/* disable clip */
	ade_clip_disable(acrtc, ch);

	/* disable scale */
	ade_scale_disable(acrtc, ch);

	/* disable ctran */
	ade_ctran_disable(acrtc, ch);

	/* disable overlay routing */
	ade_overlay_disable(acrtc, ch, zpos);

	acrtc->ch_mask &= ~BIT(ch);
	DRM_DEBUG_DRIVER("exit success.\n");
}

void ade_plane_atomic_update(struct hisi_plane *hplane,
			     struct drm_plane_state *old_state)
{
	struct drm_plane *plane = &hplane->base;
	struct drm_plane_state	*state	= plane->state;
	struct hisi_crtc *hcrtc = to_hisi_crtc(state->crtc);
	struct ade_crtc *acrtc = to_ade_crtc(hcrtc);;

	ade_update_channel(hplane, acrtc, state->fb,
		      state->crtc_x, state->crtc_y,
		      state->crtc_w, state->crtc_h,
		      state->src_x >> 16, state->src_y >> 16,
		      state->src_w >> 16, state->src_h >> 16);
}

void ade_plane_atomic_disable(struct hisi_plane *hplane,
			      struct drm_plane_state *old_state)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(old_state->crtc);
	struct ade_crtc *acrtc = to_ade_crtc(hcrtc);;

	ade_disable_channel(hplane, acrtc);
}

#define ADE_CHANNEL_SCALE_UNSUPPORT 0
#define ADE_CHANNEL_SCALE_SUPPORT 1

static const struct drm_prop_enum_list ade_ch_scale_enum_list[] = {
	{ ADE_CHANNEL_SCALE_UNSUPPORT, "unsupport" },
	{ ADE_CHANNEL_SCALE_SUPPORT, "support" },
};

static const struct drm_prop_enum_list ade_ch_blending_enum_list[] = {
	{ ALPHA_BLENDING_NONE, "blending none" },
	{ ALPHA_BLENDING_PREMULT, "blending premult" },
	{ ALPHA_BLENDING_COVERAGE, "blending coverage" }
};

static const struct drm_prop_enum_list ade_rotation_enum_list[] = {
	{ DRM_ROTATE_0,   "rotate-0" },
	{ DRM_ROTATE_90,  "rotate-90" },
	{ DRM_ROTATE_180, "rotate-180" },
	{ DRM_ROTATE_270, "rotate-270" },
	{ DRM_REFLECT_X,  "reflect-x" },
	{ DRM_REFLECT_Y,  "reflect-y" },
};

static const struct drm_prop_enum_list ade_composition_type_enum_list[] = {
	{ COMPOSITION_UNKNOWN, "composition unknown" },
	{ COMPOSITION_GLES, "composition GLES" },
	{ COMPOSITION_HWC, "composition hwc" },
	{ COMPOSITION_MIXED, "composition mixed" }
};

int ade_install_plane_properties(struct drm_device *dev,
					struct hisi_plane *hplane)
{
	struct hisi_drm_private *priv = dev->dev_private;
	struct drm_mode_object *obj = &hplane->base.base;
	struct drm_property *prop;
	u8 ch = hplane->ch;
	u64 prop_val;

	/*
	 * create and attach scale capablity properties
	 */
	prop = priv->cap_scl_prop;
	if (!prop) {
		prop = drm_property_create_enum(dev, DRM_MODE_PROP_IMMUTABLE,
				"cap_scl",
				ade_ch_scale_enum_list,
				ARRAY_SIZE(ade_ch_scale_enum_list));
		if (!prop)
			return 0;

		priv->cap_scl_prop = prop;
	}

	switch(ch) {
	case ADE_CH5:
	case ADE_CH6:
		prop_val = ADE_CHANNEL_SCALE_SUPPORT;
		break;
	default:
		prop_val = ADE_CHANNEL_SCALE_UNSUPPORT;
		break;
	}
	drm_object_attach_property(obj, prop, prop_val);


	/*
	 * create and attach rotation capablity properties
	 */
	prop = priv->cap_rot_prop;
	if (!prop) {
		prop = drm_property_create_bitmask(dev, 0, "cap_rot",
				ade_rotation_enum_list,
				ARRAY_SIZE(ade_rotation_enum_list),
				BIT(DRM_ROTATE_0) | BIT(DRM_ROTATE_90) |
				BIT(DRM_ROTATE_180) | BIT(DRM_ROTATE_270) |
				BIT(DRM_REFLECT_X) | BIT(DRM_REFLECT_Y));
		if (!prop)
			return 0;

		priv->cap_rot_prop = prop;
	}

	switch(ch) {
	case ADE_CH5:
	case ADE_CH6:
		prop_val = BIT(DRM_ROTATE_0) | BIT(DRM_ROTATE_90) |
			BIT(DRM_ROTATE_180) | BIT(DRM_ROTATE_270) |
			BIT(DRM_REFLECT_X) | BIT(DRM_REFLECT_Y);
		break;
	default:
		prop_val = BIT(DRM_ROTATE_0);
		break;
	}
	drm_object_attach_property(obj, prop, prop_val);


	/*
	 * create and attach zpos properties
	 */
	prop = priv->zpos_prop;
	if (!prop) {
		prop = drm_property_create_range(dev, 0, "zpos", 0,
						ADE_CH_NUM - 1);
		if (!prop)
			return 0;

		priv->zpos_prop = prop;
	}
	drm_object_attach_property(obj, prop, ch);

	/*
	 * create and attach rotation properties
	 */
	prop = dev->mode_config.rotation_property;
	if (!prop) {
		prop = drm_mode_create_rotation_property(dev,
				BIT(DRM_ROTATE_0) | BIT(DRM_ROTATE_90) |
				BIT(DRM_ROTATE_180) | BIT(DRM_ROTATE_270) |
				BIT(DRM_REFLECT_X) | BIT(DRM_REFLECT_Y));
		if (!prop)
			return 0;
		dev->mode_config.rotation_property = prop;
	}
	drm_object_attach_property(obj, prop, DRM_ROTATE_0);

	/*
	 * create and attach alpha properties
	 */
	prop = priv->alpha_prop;
	if (!prop) {
		prop = drm_property_create_range(dev, 0, "alpha", 0, 255);
		if (!prop)
			return 0;

		priv->alpha_prop = prop;
	}
	drm_object_attach_property(obj, prop, 255);

	/*
	 * create and attach blending properties
	 */
	prop = priv->blend_prop;
	if (!prop) {
		prop = drm_property_create_enum(dev, DRM_MODE_PROP_IMMUTABLE,
				"blend",
				ade_ch_blending_enum_list,
				ARRAY_SIZE(ade_ch_blending_enum_list));
		if (!prop)
			return 0;

		priv->blend_prop = prop;
	}
	drm_object_attach_property(obj, prop, ALPHA_BLENDING_NONE);

	return 0;

}

int ade_install_crtc_properties(struct drm_device *dev,
				struct hisi_crtc *hcrtc)
{
	struct hisi_drm_private *priv = dev->dev_private;
	struct drm_mode_object *obj = &hcrtc->base.base;
	struct drm_property *prop;

	/*
	 * create and attach composition type properties
	 */
	prop = priv->comp_type_prop;
	if (!prop) {
		prop = drm_property_create_enum(dev, 0,	"comp_type",
				ade_composition_type_enum_list,
				ARRAY_SIZE(ade_composition_type_enum_list));
		if (!prop)
			return 0;

		priv->comp_type_prop = prop;
	}
	drm_object_attach_property(obj, prop, COMPOSITION_UNKNOWN);

	return 0;
}
static struct hisi_crtc_ops ade_crtc_ops = {
	.enable = ade_crtc_enable,
	.disable = ade_crtc_disable,
	.mode_prepare = ade_crtc_mode_prepare,
	.mode_fixup = ade_crtc_mode_fixup,
	.mode_set_nofb = ade_crtc_mode_set_nofb,
	.atomic_begin = ade_crtc_atomic_begin,
	.atomic_flush = ade_crtc_atomic_flush,
	.irq_handler = ade_irq_handler,
	.enable_vblank = ade_enable_vblank,
	.disable_vblank = ade_disable_vblank,
	.install_properties = ade_install_crtc_properties,
};

static struct hisi_plane_funcs ade_plane_ops = {
	.get_properties = ade_get_channel_formats,
	.install_properties = ade_install_plane_properties,
	.atomic_update = ade_plane_atomic_update,
	.atomic_disable = ade_plane_atomic_disable,
};

static int ade_bind(struct device *dev, struct device *master, void *data)
{
	struct hisi_ade *ade = dev_get_drvdata(dev);
	struct ade_hardware_context *ctx = &ade->ctx;
	struct hisi_crtc *hcrtc = &ade->acrtc.base;
	struct drm_device *drm_dev = (struct drm_device *)data;
	struct hisi_plane *hplane;
	enum drm_plane_type type;
	int ret;
	int i;
	
	/*
	 * plane init
	 * TODO: Now only support primary plane, overlay planes
	 * need to do.
	 */
	for (i = 0; i < 1; i++) {
		hplane = &ade->hplane[i];
		hplane->ch = i;
		hplane->ctx = ctx;
		hplane->ops = &ade_plane_ops;
		type = i == PRIMARY_CH ? DRM_PLANE_TYPE_PRIMARY :
			DRM_PLANE_TYPE_OVERLAY;

		ret = hisi_drm_plane_init(drm_dev, hplane, type);
		if (ret)
			return ret;
	}

	/* crtc init */
	hcrtc->ops = &ade_crtc_ops;
	hcrtc->ctx = ctx;
	ret = hisi_drm_crtc_init(drm_dev, hcrtc, &ade->hplane[PRIMARY_CH].base);
	if (ret)
		return ret;

	/* ldi irq install */
	ret = hisi_drm_crtc_irq_install(drm_dev, ctx->irq, DRIVER_IRQ_SHARED,
					hcrtc);
	if (ret)
		return ret;

	return 0;
}

static void ade_unbind(struct device *dev, struct device *master,
	void *data)
{
	/* do nothing */
}

static const struct component_ops ade_ops = {
	.bind	= ade_bind,
	.unbind	= ade_unbind,
};

static int ade_probe(struct platform_device *pdev)
{
	struct hisi_ade *ade;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");

	ade = devm_kzalloc(&pdev->dev, sizeof(*ade), GFP_KERNEL);
	if (!ade) {
		DRM_ERROR("failed to alloc hisi_ade\n");
		return -ENOMEM;
	}

	ret = ade_dts_parse(pdev, &ade->ctx);
	if (ret) {
		DRM_ERROR("failed to parse dts!!\n");
		return ret;
	}
	
	platform_set_drvdata(pdev, ade);

	return component_add(&pdev->dev, &ade_ops);
	
	DRM_DEBUG_DRIVER("drm_ade exit successfully.\n");
}

static int ade_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &ade_ops);

	return 0;
}

static struct of_device_id ade_of_match[] = {
	{ .compatible = "hisilicon,hi6220-ade" },
	{ }
};
MODULE_DEVICE_TABLE(of, ade_of_match);

static struct platform_driver ade_driver = {
	.probe = ade_probe,
	.remove = ade_remove,
	.driver = {
		   .name = "hisi-ade",
		   .owner = THIS_MODULE,
		   .of_match_table = ade_of_match,
	},
};

module_platform_driver(ade_driver);

MODULE_AUTHOR("Xinwei Kong <kong.kongxinwei@hisilicon.com>");
MODULE_AUTHOR("Xinliang Liu <z.liuxinliang@huawei.com>");
MODULE_DESCRIPTION("hisilicon SoC DRM driver");
MODULE_LICENSE("GPL v2");
