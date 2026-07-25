// SPDX-License-Identifier: GPL-2.0 or MIT
/*
 * Copyright (c) 2025-2026 Francesco Valla <francesco@valla.it>
 *
 * Simple one-shot boot splash: draws a logo once at boot and never redraws.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/types.h>

#include <drm/drm_client.h>
#include <drm/drm_drv.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>

#include "drm_internal.h"

#define LOGO_WIDTH  240
#define LOGO_HEIGHT 320
#define LOGO_BPP    3 /* bytes per pixel (RGB888) */

struct drm_splash {
	struct drm_client_dev client;
	struct drm_client_buffer *buffer;
};

static struct drm_splash *client_to_drm_splash(struct drm_client_dev *client)
{
	return container_of(client, struct drm_splash, client);
}

static bool drm_splash_format_supported(struct drm_plane *plane, u32 format)
{
	int i;

	for (i = 0; i < plane->format_count; i++)
		if (plane->format_types[i] == format)
			return true;

	return false;
}

static int drm_splash_draw(struct drm_splash *splash,
			   unsigned int width, unsigned int height)
{
	#include "logo_kernel.h"
	unsigned int lines, bytes_per_line;
	const unsigned char *src;
	unsigned char *dst;
	struct iosys_map map;
	int ret, y;

	if (!splash->buffer)
		return -ENODEV;

	ret = drm_client_buffer_vmap(splash->buffer, &map);
	if (ret)
		return ret;

	dst = map.vaddr;
	src = logo_kernel;
	lines = min_t(unsigned int, height, LOGO_HEIGHT);
	bytes_per_line = min_t(unsigned int, width, LOGO_WIDTH) * LOGO_BPP;

	for (y = 0; y < lines; y++)
		memcpy(dst + y * splash->buffer->pitch,
		       src + y * LOGO_WIDTH * LOGO_BPP,
		       bytes_per_line);

	drm_client_buffer_vunmap(splash->buffer);
	return 0;
}

static int drm_splash_show(struct drm_splash *splash)
{
	struct drm_client_dev *client = &splash->client;
	struct drm_mode_set *modeset;
	unsigned int width, height;
	int ret;

	ret = drm_client_modeset_probe(client, 0, 0);
	if (ret)
		return ret;

	/* Find the first usable modeset */
	drm_client_for_each_modeset(modeset, client) {
		if (!modeset->mode)
			continue;
		if (!drm_splash_format_supported(modeset->crtc->primary,
						 DRM_FORMAT_RGB888))
			continue;

		width = modeset->mode->hdisplay;
		height = modeset->mode->vdisplay;

		splash->buffer = drm_client_framebuffer_create(client,
							       width, height,
							       DRM_FORMAT_RGB888);
		if (IS_ERR(splash->buffer)) {
			splash->buffer = NULL;
			drm_warn(client->dev,
				 "splash: failed to create framebuffer %ux%u\n",
				 width, height);
			continue;
		}

		ret = drm_splash_draw(splash, width, height);
		if (ret) {
			drm_client_framebuffer_delete(splash->buffer);
			splash->buffer = NULL;
			drm_err(client->dev,
				"splash: failed to draw logo: %d\n", ret);
			continue;
		}

		modeset->fb = splash->buffer->fb;

		ret = drm_client_modeset_commit(client);
		if (ret)
			drm_err(client->dev,
				"splash: modeset commit failed: %d\n", ret);
		else
			drm_info(client->dev,
				 "splash: displayed %ux%u logo\n",
				 width, height);

		/* One-shot: we're done regardless of success */
		return ret;
	}

	drm_info(client->dev, "splash: no usable modeset found\n");
	return -ENODEV;
}

static void drm_splash_client_unregister(struct drm_client_dev *client)
{
	struct drm_splash *splash = client_to_drm_splash(client);

	if (splash->buffer)
		drm_client_framebuffer_delete(splash->buffer);
	drm_client_release(client);
	kfree(splash);
}

static const struct drm_client_funcs drm_splash_client_funcs = {
	.owner		= THIS_MODULE,
	.unregister	= drm_splash_client_unregister,
};

/**
 * drm_splash_register() - Register a drm device to drm_splash
 * @dev: the drm device to register.
 *
 * Draws a one-shot boot splash logo. The framebuffer persists on screen
 * until another DRM master (e.g. a userspace app) takes over.
 */
void drm_splash_register(struct drm_device *dev)
{
	struct drm_splash *splash;
	int ret;

	splash = kzalloc(sizeof(*splash), GFP_KERNEL);
	if (!splash)
		goto err_warn;

	ret = drm_client_init(dev, &splash->client, "drm_splash", NULL);
	if (ret)
		goto err_free;

	drm_client_register(&splash->client);

	ret = drm_splash_show(splash);
	if (ret)
		drm_info(dev, "splash: skipped (%d)\n", ret);

	return;

err_free:
	kfree(splash);
err_warn:
	drm_warn(dev, "splash: failed to register\n");
}
EXPORT_SYMBOL(drm_splash_register);
