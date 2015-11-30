/*
 * This file is part of MTD RAID.
 *
 * Copyright (C) 2015 Dongsheng Yang. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Authors: Dongsheng Yang <yangds.fnst@cn.fujitsu.com>
 */

/**
 * This file implement raid0.
 * RAID0:
 *
 * Flash A:
 * 	------------------------------
 * 	|  A1   |  A2   | ... |  An  |
 * 	------------------------------
 * Flash B:
 * 	------------------------------
 * 	|  B1   |  B2   | ... |  Bn  |
 * 	------------------------------
 * Flash C:
 * 	------------------------------
 * 	|  C1   |  C2   | ... |  Cn  |
 * 	------------------------------
 *
 * RAID0:
 * 	-----------------------------------------------------------------------
 * 	|  A1   |  B1  |  C1  |  A2  |  B2  |  C2  | ... |  An  |  Bn  |  Cn  |
 * 	-----------------------------------------------------------------------
 *
 * Detail to see:
 * https://en.wikipedia.org/wiki/Standard_RAID_levels
 */

#include <linux/slab.h>
#include <linux/kthread.h>

#include "mtd_raid.h"

static int raid0_logical_to_physical(struct mtd_raid *raid, loff_t from, size_t len, int copy_num,
			      int *devid, loff_t *subdev_off, size_t *size)
{
	loff_t stripe_size, stripe_offs, stripe_num, substripe_offs, substripe_num;

	stripe_size = raid->substripe_size * raid->npebs_per_leb;
	stripe_offs = do_div(from, stripe_size);
	stripe_num = from;
	substripe_offs = do_div(stripe_offs, raid->substripe_size);
	substripe_num = stripe_offs;

	*devid = substripe_num;
	*subdev_off = stripe_num * raid->substripe_size + substripe_offs;
	*size = len;
	if (*size > raid->substripe_size - substripe_offs)
		*size = raid->substripe_size - substripe_offs;

	return 0;
}

struct mtd_raid *mtd_raid0_create(int dev_count, size_t substripe_size)
{
	struct mtd_raid0 *raid0 = NULL;
	struct mtd_raid *raid = NULL;

	raid0 = kzalloc(sizeof(struct mtd_raid0) + sizeof(struct mtd_raid_dev) * dev_count, GFP_KERNEL);
	if (!raid0)
		goto out;

	raid = &raid0->raid;
	raid->raid_level = MTD_RAID_LEVEL_RAID0;
	raid->ops = &mtd_raid0_ops;
	raid->ncopies = 1;
	raid->npebs_per_leb = dev_count;

	return &raid0->raid;
out:
	return NULL;
}

static int raid0_init(struct mtd_raid *raid,
			    int dev_count, size_t substripe_size)
{
	int ret = 0;
	int i = 0;
	int raid_id = 0;
	struct mtd_info *mtd = &raid->mtd;
	struct mtd_info *subdev0 = raid->devs[0].mtd;
	struct mtd_info *subdev = NULL;

	raid_id = mtd_raid_list_register(MTD_RAID_LEVEL_RAID0, raid);
	sprintf(raid->name, "mtd0-%d", raid_id);
	mtd->name = raid->name;

	for (i = 0; i < dev_count; i++) {
		subdev = raid->devs[i].mtd;
		if (subdev0->size != subdev->size) {
			pr_err("Incompatible size on \"%s\"",
			       subdev->name);
			ret = -EINVAL;
			goto out;
		}

		mtd->size += subdev->size;
		mtd->erasesize += subdev->erasesize;
	}
out:
	return ret;
}


void raid0_destroy(struct mtd_raid *mtd_raid)
{
	struct mtd_raid0 *raid0 = MTD_RAID_RAID0(mtd_raid);

	mtd_raid_list_unregister(&raid0->raid);
	kfree(raid0);
}

const struct mtd_raid_operations mtd_raid0_ops = {
	.init    = raid0_init,
	.destroy = raid0_destroy,
	.logical_to_physical  = raid0_logical_to_physical,
};
