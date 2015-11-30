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

/*
 * This file implement single raild level.
 *
 * RAID_SINGLE:
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
 * 	---------------------------------------------------------------------
 * 	|  A1   |  A2  | ... |  An  |  B1  | ... |  Bn  |  C1  | ... |  Cn  |
 * 	---------------------------------------------------------------------
 *
 * It works similarly with concat.
 */
#include <linux/kthread.h>
#include <linux/slab.h>

#include "mtd_raid.h"

/**
 *
 */
static int raid_single_logical_to_physical(struct mtd_raid *raid, loff_t from, size_t len, int copy_num,
					   int *devid, loff_t *subdev_off, size_t *size)
{
	struct mtd_info *subdev = NULL;
	int i = 0;

	*subdev_off = from;
	for (i = 0; i < raid->dev_count; i++) {
		subdev = raid->devs[i].mtd;
		if (from >= subdev->size) {
			from -= subdev->size;
			*subdev_off = 0;
			*size = 0;
			continue;
		}

		*subdev_off = from;
		if (from + len > subdev->size)
			*size = subdev->size - *subdev_off;
		else
			*size = len;

		*devid = i;
		break;
	}

	if (i == raid->dev_count)
		return -EINVAL;

	return 0;
}

struct mtd_raid *mtd_raid_single_create(int *mtd_nums, int dev_count, size_t substripe_size)
{
	struct mtd_raid_single *raid_single = NULL;
	struct mtd_raid *raid = NULL;

	raid_single = kzalloc(sizeof(struct mtd_raid_single) + sizeof(struct mtd_raid_dev) * dev_count, GFP_KERNEL);
	if (!raid_single)
		goto out;

	raid = &raid_single->raid;
	raid->raid_level = MTD_RAID_LEVEL_SINGLE;
	raid->ops = &mtd_raid_single_ops;
	raid->ncopies = 1;
	raid->npebs_per_leb = 1;

	return &raid_single->raid;
out:
	return NULL;
}

static int raid_single_init(struct mtd_raid *raid,
			    int dev_count, size_t substripe_size)
{
	int ret = 0;
	int i = 0;
	int raid_id = 0;
	struct mtd_info *mtd = &raid->mtd;
	struct mtd_info *subdev = raid->devs[0].mtd;

	raid_id = mtd_raid_list_register(MTD_RAID_LEVEL_SINGLE, raid);
	sprintf(raid->name, "mtdsingle-%d", raid_id);
	mtd->name = raid->name;

	/*
	 * Same erasesize with subdevs
	 **/
	mtd->erasesize = subdev->erasesize;
	for (i = 0; i < dev_count; i++) {
		subdev = raid->devs[i].mtd;
		if (mtd->erasesize != subdev->erasesize) {
			pr_err("Incompatible erasesize on \"%s\"",
			       subdev->name);
			ret = -EINVAL;
			goto err;
		}
		/*
		 * mtd->size is the sum of all subdevs
		 **/
		mtd->size += subdev->size;
	}
	return ret;
err:
	mtd_raid_list_unregister(raid);
	return ret;
}


static void raid_single_destroy(struct mtd_raid *mtd_raid)
{
	struct mtd_raid_single *raid_single = MTD_RAID_SINGLE(mtd_raid);

	mtd_raid_list_unregister(&raid_single->raid);
	kfree(raid_single);
}

const struct mtd_raid_operations mtd_raid_single_ops = {
	.init    = raid_single_init,
	.destroy = raid_single_destroy,
	.logical_to_physical  = raid_single_logical_to_physical,
};
