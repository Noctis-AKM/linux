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
 * This file implement raid1.
 * RAID1:
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
 * RAID_SINGLE:
 * 	------------------------------
 * 	|  A1   |  A2   | ... |  An  |
 * 	------------------------------
 * 	------------------------------
 * 	|  B1   |  B2   | ... |  Bn  | Mirror
 * 	------------------------------
 * 	------------------------------
 * 	|  C1   |  C2   | ... |  Cn  | Mirror
 * 	------------------------------
 *
 * C1 = B1 = A1. They store same data in three different flashes.
 *
 * Detail to see:
 * https://en.wikipedia.org/wiki/Standard_RAID_levels
 */

#include <linux/slab.h>
#include <linux/kthread.h>

#include "mtd_raid.h"

static int raid1_logical_to_physical(struct mtd_raid *raid, loff_t from, size_t len, int copy_num,
			      int *devid, loff_t *subdev_off, size_t *size)
{
	*devid = copy_num;
	*subdev_off = from;
	*size = len;

	return 0;
}

/**
 * This function will convert the physical address to logical address.
 */
static int raid1_physical_to_logical(struct mtd_raid *raid, int devid, loff_t subdev_off, size_t size,
				     loff_t *from, size_t *len)
{
	*from = subdev_off;
	*len = size;

	return 0;
}

struct mtd_raid *mtd_raid1_create(int *mtd_nums, int dev_count, size_t substripe_size)
{
	struct mtd_raid1 *raid1 = NULL;
	struct mtd_raid *raid = NULL;

	raid1 = kzalloc(sizeof(struct mtd_raid1) + sizeof(struct mtd_raid_dev) * dev_count, GFP_KERNEL);
	if (!raid1)
		goto out;

	raid = &raid1->raid;
	raid->raid_level = MTD_RAID_LEVEL_RAID1;
	raid->ops = &mtd_raid1_ops;
	raid->ncopies = dev_count;
	raid->npebs_per_leb = 1;

	return &raid1->raid;
out:
	return NULL;
}

static int raid1_init(struct mtd_raid *raid,
			    int dev_count, size_t substripe_size)
{
	int ret = 0;
	int i = 0;
	int raid_id = 0;
	struct mtd_info *mtd = &raid->mtd;
	struct mtd_info *subdev = raid->devs[0].mtd;

	raid_id = mtd_raid_list_register(MTD_RAID_LEVEL_RAID1, raid);
	sprintf(raid->name, "mtd1-%d", raid_id);
	mtd->name = raid->name;

	/**
	 * size and erasesize are same with subdev.
	 */
	mtd->size = subdev->size;
	mtd->erasesize = subdev->erasesize;
	for (i = 0; i < dev_count; i++) {
		subdev = raid->devs[i].mtd;
		if (mtd->size != subdev->size) {
			pr_err("Incompatible size on \"%s\"",
			       subdev->name);
			ret = -EINVAL;
			goto out;
		}

		if (mtd->erasesize != subdev->erasesize) {
			pr_err("Incompatible erasesize on \"%s\"",
			       subdev->name);
			ret = -EINVAL;
			goto out;
		}
	}

out:
	return ret;
}


void raid1_destroy(struct mtd_raid *mtd_raid)
{
	struct mtd_raid1 *raid1 = MTD_RAID_RAID1(mtd_raid);

	mtd_raid_list_unregister(&raid1->raid);
	kfree(raid1);
}

const struct mtd_raid_operations mtd_raid1_ops = {
	.init    = raid1_init,
	.destroy = raid1_destroy,
	.logical_to_physical = raid1_logical_to_physical,
	.physical_to_logical = raid1_physical_to_logical,
};
