/*
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

/**
 * @modes: Pointer to array of fixed modes appropriate for this panel.  If
 *         only one mode then this can just be the address of this the mode.
 *         NOTE: cannot be used with "timings" and also if this is specified
 *         then you cannot override the mode in the device tree.
 * @num_modes: Number of elements in modes array.
 * @timings: Pointer to array of display timings.  NOTE: cannot be used with
 *           "modes" and also these will be used to validate a device tree
 *           override if one is present.
 * @num_timings: Number of elements in timings array.
 * @bpc: Bits per color.
 * @size: Structure containing the physical size of this panel.
 * @delay: Structure containing various delay values for this panel.
 * @bus_format: See MEDIA_BUS_FMT_... defines.
 * @bus_flags: See DRM_BUS_FLAG_... defines.
 */
struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct display_timing *timings;
	unsigned int num_timings;

	unsigned int bpc;

	/**
	 * @width: width (in millimeters) of the panel's active display area
	 * @height: height (in millimeters) of the panel's active display area
	 */
	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @hpd_absent_delay: Add this to the prepare delay if we know Hot
	 *                    Plug Detect isn't used.
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int hpd_absent_delay;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;

	u32 bus_format;
	u32 bus_flags;
	int connector_type;
};

struct panel_simple {
	struct drm_panel base;
	bool prepared;
	bool enabled;
	bool no_hpd;

	const struct panel_desc *desc;

	struct regulator *supply;
	struct i2c_adapter *ddc;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *hpd_gpio;

	struct drm_display_mode override_mode;

	enum drm_panel_orientation orientation;
};

static inline struct panel_simple *to_panel_simple(struct drm_panel *panel)
{
	return container_of(panel, struct panel_simple, base);
}

static unsigned int panel_simple_get_timings_modes(struct panel_simple *panel,
						   struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	for (i = 0; i < panel->desc->num_timings; i++) {
		const struct display_timing *dt = &panel->desc->timings[i];
		struct videomode vm;

		videomode_from_timing(dt, &vm);
		mode = drm_mode_create(connector->dev);
		if (!mode) {
			dev_err(panel->base.dev, "failed to add mode %ux%u\n",
				dt->hactive.typ, dt->vactive.typ);
			continue;
		}

		drm_display_mode_from_videomode(&vm, mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_timings == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
		num++;
	}

	return num;
}

static unsigned int panel_simple_get_display_modes(struct panel_simple *panel,
						   struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	for (i = 0; i < panel->desc->num_modes; i++) {
		const struct drm_display_mode *m = &panel->desc->modes[i];

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->base.dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay,
				drm_mode_vrefresh(m));
			continue;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	return num;
}

static int panel_simple_get_non_edid_modes(struct panel_simple *panel,
					   struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	bool has_override = panel->override_mode.type;
	unsigned int num = 0;

	if (!panel->desc)
		return 0;

	if (has_override) {
		mode = drm_mode_duplicate(connector->dev,
					  &panel->override_mode);
		if (mode) {
			drm_mode_probed_add(connector, mode);
			num = 1;
		} else {
			dev_err(panel->base.dev, "failed to add override mode\n");
		}
	}

	/* Only add timings if override was not there or failed to validate */
	if (num == 0 && panel->desc->num_timings)
		num = panel_simple_get_timings_modes(panel, connector);

	/*
	 * Only add fixed modes if timings/override added no mode.
	 *
	 * We should only ever have either the display timings specified
	 * or a fixed mode. Anything else is rather bogus.
	 */
	WARN_ON(panel->desc->num_timings && panel->desc->num_modes);
	if (num == 0)
		num = panel_simple_get_display_modes(panel, connector);

	connector->display_info.bpc = panel->desc->bpc;
	connector->display_info.width_mm = panel->desc->size.width;
	connector->display_info.height_mm = panel->desc->size.height;
	if (panel->desc->bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
						 &panel->desc->bus_format, 1);
	connector->display_info.bus_flags = panel->desc->bus_flags;

	return num;
}

static int panel_simple_disable(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);

	if (!p->enabled)
		return 0;

	if (p->desc->delay.disable)
		msleep(p->desc->delay.disable);

	p->enabled = false;

	return 0;
}

static int panel_simple_unprepare(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);

	if (!p->prepared)
		return 0;

	gpiod_set_value_cansleep(p->enable_gpio, 0);

	regulator_disable(p->supply);

	if (p->desc->delay.unprepare)
		msleep(p->desc->delay.unprepare);

	p->prepared = false;

	return 0;
}

static int panel_simple_get_hpd_gpio(struct device *dev,
				     struct panel_simple *p, bool from_probe)
{
	int err;

	p->hpd_gpio = devm_gpiod_get_optional(dev, "hpd", GPIOD_IN);
	if (IS_ERR(p->hpd_gpio)) {
		err = PTR_ERR(p->hpd_gpio);

		/*
		 * If we're called from probe we won't consider '-EPROBE_DEFER'
		 * to be an error--we'll leave the error code in "hpd_gpio".
		 * When we try to use it we'll try again.  This allows for
		 * circular dependencies where the component providing the
		 * hpd gpio needs the panel to init before probing.
		 */
		if (err != -EPROBE_DEFER || !from_probe) {
			dev_err(dev, "failed to get 'hpd' GPIO: %d\n", err);
			return err;
		}
	}

	return 0;
}

static int panel_simple_prepare(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	unsigned int delay;
	int err;
	int hpd_asserted;

	if (p->prepared)
		return 0;

	err = regulator_enable(p->supply);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable supply: %d\n", err);
		return err;
	}

	gpiod_set_value_cansleep(p->enable_gpio, 1);

	delay = p->desc->delay.prepare;
	if (p->no_hpd)
		delay += p->desc->delay.hpd_absent_delay;
	if (delay)
		msleep(delay);

	if (p->hpd_gpio) {
		if (IS_ERR(p->hpd_gpio)) {
			err = panel_simple_get_hpd_gpio(panel->dev, p, false);
			if (err)
				return err;
		}

		err = readx_poll_timeout(gpiod_get_value_cansleep, p->hpd_gpio,
					 hpd_asserted, hpd_asserted,
					 1000, 2000000);
		if (hpd_asserted < 0)
			err = hpd_asserted;

		if (err) {
			dev_err(panel->dev,
				"error waiting for hpd GPIO: %d\n", err);
			return err;
		}
	}

	p->prepared = true;

	return 0;
}

static int panel_simple_enable(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);

	if (p->enabled)
		return 0;

	if (p->desc->delay.enable)
		msleep(p->desc->delay.enable);

	p->enabled = true;

	return 0;
}

static int panel_simple_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	struct panel_simple *p = to_panel_simple(panel);
	int num = 0;

	/* probe EDID if a DDC bus is available */
	if (p->ddc) {
		struct edid *edid = drm_get_edid(connector, p->ddc);

		drm_connector_update_edid_property(connector, edid);
		if (edid) {
			num += drm_add_edid_modes(connector, edid);
			kfree(edid);
		}
	}

	/* add hard-coded panel modes */
	num += panel_simple_get_non_edid_modes(p, connector);

	/* set up connector's "panel orientation" property */
	drm_connector_set_panel_orientation(connector, p->orientation);

	return num;
}

static int panel_simple_get_timings(struct drm_panel *panel,
				    unsigned int num_timings,
				    struct display_timing *timings)
{
	struct panel_simple *p = to_panel_simple(panel);
	unsigned int i;

	if (p->desc->num_timings < num_timings)
		num_timings = p->desc->num_timings;

	if (timings)
		for (i = 0; i < num_timings; i++)
			timings[i] = p->desc->timings[i];

	return p->desc->num_timings;
}

static const struct drm_panel_funcs panel_simple_funcs = {
	.disable = panel_simple_disable,
	.unprepare = panel_simple_unprepare,
	.prepare = panel_simple_prepare,
	.enable = panel_simple_enable,
	.get_modes = panel_simple_get_modes,
	.get_timings = panel_simple_get_timings,
};

static struct panel_desc panel_dpi;

static int panel_dpi_probe(struct device *dev,
			   struct panel_simple *panel)
{
	struct display_timing *timing;
	const struct device_node *np;
	struct panel_desc *desc;
	unsigned int bus_flags;
	struct videomode vm;
	int ret;

	np = dev->of_node;
	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	timing = devm_kzalloc(dev, sizeof(*timing), GFP_KERNEL);
	if (!timing)
		return -ENOMEM;

	ret = of_get_display_timing(np, "panel-timing", timing);
	if (ret < 0) {
		dev_err(dev, "%pOF: no panel-timing node found for \"panel-dpi\" binding\n",
			np);
		return ret;
	}

	desc->timings = timing;
	desc->num_timings = 1;

	of_property_read_u32(np, "width-mm", &desc->size.width);
	of_property_read_u32(np, "height-mm", &desc->size.height);

	/* Extract bus_flags from display_timing */
	bus_flags = 0;
	vm.flags = timing->flags;
	drm_bus_flags_from_videomode(&vm, &bus_flags);
	desc->bus_flags = bus_flags;

	/* We do not know the connector for the DT node, so guess it */
	desc->connector_type = DRM_MODE_CONNECTOR_DPI;

	panel->desc = desc;

	return 0;
}

#define PANEL_SIMPLE_BOUNDS_CHECK(to_check, bounds, field) \
	(to_check->field.typ >= bounds->field.min && \
	 to_check->field.typ <= bounds->field.max)
static void panel_simple_parse_panel_timing_node(struct device *dev,
						 struct panel_simple *panel,
						 const struct display_timing *ot)
{
	const struct panel_desc *desc = panel->desc;
	struct videomode vm;
	unsigned int i;

	if (WARN_ON(desc->num_modes)) {
		dev_err(dev, "Reject override mode: panel has a fixed mode\n");
		return;
	}
	if (WARN_ON(!desc->num_timings)) {
		dev_err(dev, "Reject override mode: no timings specified\n");
		return;
	}

	for (i = 0; i < panel->desc->num_timings; i++) {
		const struct display_timing *dt = &panel->desc->timings[i];

		if (!PANEL_SIMPLE_BOUNDS_CHECK(ot, dt, hactive) ||
		    !PANEL_SIMPLE_BOUNDS_CHECK(ot, dt, hfront_porch) ||
		    !PANEL_SIMPLE_BOUNDS_CHECK(ot, dt, hback_porch) ||
		    !PANEL_SIMPLE_BOUNDS_CHECK(ot, dt, hsync_len) ||
		    !PANEL_SIMPLE_BOUNDS_CHECK(ot, dt, vactive) ||
		    !PANEL_SIMPLE_BOUNDS_CHECK(ot, dt, vfront_porch) ||
		    !PANEL_SIMPLE_BOUNDS_CHECK(ot, dt, vback_porch) ||
		    !PANEL_SIMPLE_BOUNDS_CHECK(ot, dt, vsync_len))
			continue;

		if (ot->flags != dt->flags)
			continue;

		videomode_from_timing(ot, &vm);
		drm_display_mode_from_videomode(&vm, &panel->override_mode);
		panel->override_mode.type |= DRM_MODE_TYPE_DRIVER |
					     DRM_MODE_TYPE_PREFERRED;
		break;
	}

	if (WARN_ON(!panel->override_mode.type))
		dev_err(dev, "Reject override mode: No display_timing found\n");
}

static int panel_simple_probe(struct device *dev, const struct panel_desc *desc)
{
	struct panel_simple *panel;
	struct display_timing dt;
	struct device_node *ddc;
	int connector_type;
	u32 bus_flags;
	int err;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->enabled = false;
	panel->prepared = false;
	panel->desc = desc;

	panel->no_hpd = of_property_read_bool(dev->of_node, "no-hpd");
	if (!panel->no_hpd) {
		err = panel_simple_get_hpd_gpio(dev, panel, true);
		if (err)
			return err;
	}

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	panel->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						     GPIOD_OUT_LOW);
	if (IS_ERR(panel->enable_gpio)) {
		err = PTR_ERR(panel->enable_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "failed to request GPIO: %d\n", err);
		return err;
	}

	err = of_drm_get_panel_orientation(dev->of_node, &panel->orientation);
	if (err) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, err);
		return err;
	}

	ddc = of_parse_phandle(dev->of_node, "ddc-i2c-bus", 0);
	if (ddc) {
		panel->ddc = of_find_i2c_adapter_by_node(ddc);
		of_node_put(ddc);

		if (!panel->ddc)
			return -EPROBE_DEFER;
	}

	if (desc == &panel_dpi) {
		/* Handle the generic panel-dpi binding */
		err = panel_dpi_probe(dev, panel);
		if (err)
			goto free_ddc;
	} else {
		if (!of_get_display_timing(dev->of_node, "panel-timing", &dt))
			panel_simple_parse_panel_timing_node(dev, panel, &dt);
	}

	connector_type = desc->connector_type;
	/* Catch common mistakes for panels. */
	switch (connector_type) {
	case 0:
		dev_warn(dev, "Specify missing connector_type\n");
		connector_type = DRM_MODE_CONNECTOR_DPI;
		break;
	case DRM_MODE_CONNECTOR_LVDS:
		WARN_ON(desc->bus_flags &
			~(DRM_BUS_FLAG_DE_LOW |
			  DRM_BUS_FLAG_DE_HIGH |
			  DRM_BUS_FLAG_DATA_MSB_TO_LSB |
			  DRM_BUS_FLAG_DATA_LSB_TO_MSB));
		WARN_ON(desc->bus_format != MEDIA_BUS_FMT_RGB666_1X7X3_SPWG &&
			desc->bus_format != MEDIA_BUS_FMT_RGB888_1X7X4_SPWG &&
			desc->bus_format != MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA);
		WARN_ON(desc->bus_format == MEDIA_BUS_FMT_RGB666_1X7X3_SPWG &&
			desc->bpc != 6);
		WARN_ON((desc->bus_format == MEDIA_BUS_FMT_RGB888_1X7X4_SPWG ||
			 desc->bus_format == MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA) &&
			desc->bpc != 8);
		break;
	case DRM_MODE_CONNECTOR_eDP:
		if (desc->bus_format == 0)
			dev_warn(dev, "Specify missing bus_format\n");
		if (desc->bpc != 6 && desc->bpc != 8)
			dev_warn(dev, "Expected bpc in {6,8} but got: %u\n", desc->bpc);
		break;
	case DRM_MODE_CONNECTOR_DSI:
		if (desc->bpc != 6 && desc->bpc != 8)
			dev_warn(dev, "Expected bpc in {6,8} but got: %u\n", desc->bpc);
		break;
	case DRM_MODE_CONNECTOR_DPI:
		bus_flags = DRM_BUS_FLAG_DE_LOW |
			    DRM_BUS_FLAG_DE_HIGH |
			    DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE |
			    DRM_BUS_FLAG_PIXDATA_SAMPLE_NEGEDGE |
			    DRM_BUS_FLAG_DATA_MSB_TO_LSB |
			    DRM_BUS_FLAG_DATA_LSB_TO_MSB |
			    DRM_BUS_FLAG_SYNC_SAMPLE_POSEDGE |
			    DRM_BUS_FLAG_SYNC_SAMPLE_NEGEDGE;
		if (desc->bus_flags & ~bus_flags)
			dev_warn(dev, "Unexpected bus_flags(%d)\n", desc->bus_flags & ~bus_flags);
		if (!(desc->bus_flags & bus_flags))
			dev_warn(dev, "Specify missing bus_flags\n");
		if (desc->bus_format == 0)
			dev_warn(dev, "Specify missing bus_format\n");
		if (desc->bpc != 6 && desc->bpc != 8)
			dev_warn(dev, "Expected bpc in {6,8} but got: %u\n", desc->bpc);
		break;
	default:
		dev_warn(dev, "Specify a valid connector_type: %d\n", desc->connector_type);
		connector_type = DRM_MODE_CONNECTOR_DPI;
		break;
	}

	drm_panel_init(&panel->base, dev, &panel_simple_funcs, connector_type);

	err = drm_panel_of_backlight(&panel->base);
	if (err)
		goto free_ddc;

	drm_panel_add(&panel->base);

	dev_set_drvdata(dev, panel);

	return 0;

free_ddc:
	if (panel->ddc)
		put_device(&panel->ddc->dev);

	return err;
}

static int panel_simple_remove(struct device *dev)
{
	struct panel_simple *panel = dev_get_drvdata(dev);

	drm_panel_remove(&panel->base);
	drm_panel_disable(&panel->base);
	drm_panel_unprepare(&panel->base);

	if (panel->ddc)
		put_device(&panel->ddc->dev);

	return 0;
}

static void panel_simple_shutdown(struct device *dev)
{
	struct panel_simple *panel = dev_get_drvdata(dev);

	drm_panel_disable(&panel->base);
	drm_panel_unprepare(&panel->base);
}

// h   pol fp  sync bp  v   vpol fp  sync bp       fps i pixel_f  aspect
// 480 0   10  16   59  800 0    15  113  15 0 0 0 60  0 32000000 6 
//
// 0x7f216
// output_format = 6          DPI_OUTPUT_FORMAT_18BIT_666_CFG2
// rgb_order = 1              DPI_RGB_ORDER_RGB
// invert_pixel_clock = 1     RGB data changes on falling edge
// hsync_polarity = 1         Inverted from HDMI default (active low??)
// vsync_polarity = 1         Inverted from HDMI default (active low??)
// output_enable_polarity = 1
// hsync_phase = 0            DPI_PHASE_POSEDGE / DRM_BUS_FLAG_SYNC_SAMPLE_POSEDGE
// vsync_phase = 0            DPI_PHASE_POSEDGE / DRM_BUS_FLAG_SYNC_SAMPLE_POSEDGE
// output_enable_phase = 0    DPI_PHASE_POSEDGE
// Pixel formats: https://www.kernel.org/doc/html/v4.15/media/uapi/v4l/subdev-formats.html
// RPI formats: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#controlling-timings-and-resolutions

static const struct display_timing pimoroni_hyperpixel4_timing = {
	.pixelclock = { 31900000, 31967700, 32000000 },
	.hactive = { 480, 480, 480 },
	.hfront_porch = { 10, 10, 10 },
	.hback_porch = { 59, 59, 59 },
	.hsync_len = { 16, 16, 16 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 15, 15, 15 },
	.vback_porch = { 15, 15, 15 },
	.vsync_len = { 113, 113, 113 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW |
		 DISPLAY_FLAGS_DE_LOW | DISPLAY_FLAGS_PIXDATA_POSEDGE |
		 DISPLAY_FLAGS_SYNC_POSEDGE,
};

static const struct drm_display_mode pimoroni_hyperpixel4_mode = {
	.clock = 32000,
	.hdisplay = 480,
	.hsync_start = 480 + 10,
	.hsync_end = 480 + 10 + 59,
	.htotal = 480 + 10 + 59 + 16,
	.vdisplay = 800,
	.vsync_start = 800 + 15,
	.vsync_end = 800 + 15 + 15,
	.vtotal = 800 + 15 + 15 + 113,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC | DRM_BUS_FLAG_DE_LOW | DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE | DRM_BUS_FLAG_SYNC_SAMPLE_POSEDGE,
};

static const struct panel_desc pimoroni_hyperpixel4 = {
	//.modes = &pimoroni_hyperpixel4_mode,
	//.num_modes = 1,
	.timings = &pimoroni_hyperpixel4_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 51,  // Physical width (mm)
		.height = 85, // Physical height (mm)
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X24_CPADHI,
};

static const struct of_device_id platform_of_match[] = {
	{
		.compatible = "pimoroni,hyperpixel4",
		.data = &pimoroni_hyperpixel4,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, platform_of_match);

static int panel_simple_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return panel_simple_probe(&pdev->dev, id->data);
}

static int panel_simple_platform_remove(struct platform_device *pdev)
{
	return panel_simple_remove(&pdev->dev);
}

static void panel_simple_platform_shutdown(struct platform_device *pdev)
{
	panel_simple_shutdown(&pdev->dev);
}

static struct platform_driver panel_simple_platform_driver = {
	.driver = {
		.name = "panel-pimoroni",
		.of_match_table = platform_of_match,
	},
	.probe = panel_simple_platform_probe,
	.remove = panel_simple_platform_remove,
	.shutdown = panel_simple_platform_shutdown,
};

struct panel_desc_dsi {
	struct panel_desc desc;

	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
};

static const struct of_device_id dsi_of_match[] = {
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, dsi_of_match);

static int panel_simple_dsi_probe(struct mipi_dsi_device *dsi)
{
	const struct panel_desc_dsi *desc;
	const struct of_device_id *id;
	int err;

	id = of_match_node(dsi_of_match, dsi->dev.of_node);
	if (!id)
		return -ENODEV;

	desc = id->data;

	err = panel_simple_probe(&dsi->dev, &desc->desc);
	if (err < 0)
		return err;

	dsi->mode_flags = desc->flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	err = mipi_dsi_attach(dsi);
	if (err) {
		struct panel_simple *panel = dev_get_drvdata(&dsi->dev);

		drm_panel_remove(&panel->base);
	}

	return err;
}

static int panel_simple_dsi_remove(struct mipi_dsi_device *dsi)
{
	int err;

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", err);

	return panel_simple_remove(&dsi->dev);
}

static void panel_simple_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	panel_simple_shutdown(&dsi->dev);
}

static struct mipi_dsi_driver panel_simple_dsi_driver = {
	.driver = {
		.name = "panel-pimoroni-dsi",
		.of_match_table = dsi_of_match,
	},
	.probe = panel_simple_dsi_probe,
	.remove = panel_simple_dsi_remove,
	.shutdown = panel_simple_dsi_shutdown,
};

static int __init panel_simple_init(void)
{
	int err;

	err = platform_driver_register(&panel_simple_platform_driver);
	if (err < 0)
		return err;

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI)) {
		err = mipi_dsi_driver_register(&panel_simple_dsi_driver);
		if (err < 0)
			return err;
	}

	return 0;
}
module_init(panel_simple_init);

static void __exit panel_simple_exit(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&panel_simple_dsi_driver);

	platform_driver_unregister(&panel_simple_platform_driver);
}
module_exit(panel_simple_exit);

MODULE_AUTHOR("Philip Howard <phil@pimoroni.com>");
MODULE_DESCRIPTION("DRM Driver for Testing Simple Panels");
MODULE_LICENSE("GPL and additional rights");

