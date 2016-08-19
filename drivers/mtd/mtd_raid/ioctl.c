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

#include <linux/compat.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "mtd_raid.h"

static int get_subdevs(int *mtd_nums, struct mtd_info **subdevs, int dev_count)
{
	int i = 0;
	int ret = 0;
	struct mtd_info *subdev = NULL;

	for (i = 0; i < dev_count; i++) {
		subdev = get_mtd_device(NULL, mtd_nums[i]);
		if (IS_ERR(subdev)) {
			ret = PTR_ERR(subdev);
			pr_err("error: Cannot get MTD device. mtd_num: %d\n", mtd_nums[i]);
			break;
		}
		subdevs[i] = subdev;
	}
	return ret;
}

static void put_subdevs(struct mtd_info **subdevs, int dev_count)
{
	int i = 0;

	for (i = 0; i < dev_count; i++) {
		if (subdevs[i])
			put_mtd_device(subdevs[i]);
	}
}

static int ioctl_create_check(int *mtd_nums, int dev_count, size_t substripe_size)
{
	int i = 0;

	if (dev_count < 0 || substripe_size < 0)
		return -EINVAL;

	for (i = 0; i < dev_count; i++) {
		if (mtd_nums[i] < 0)
			return -EINVAL;
	}

	return 0;
}

int mtd_raid_ioctl_create(struct mtd_raid_create_req *create_req, int *mtd_nums)
{
	enum mtd_raid_level raid_level = create_req->raid_level;
	int dev_count = create_req->dev_count;
	int substripe_size = create_req->substripe_size;
	struct mtd_info **subdevs;
	int ret = 0;

	if (ioctl_create_check(mtd_nums, dev_count, substripe_size))
		return -EINVAL;

	subdevs = kmalloc(sizeof(struct mtd_info *) * dev_count, GFP_KERNEL);
	if (!subdevs)
		return -ENOMEM;

	ret = get_subdevs(mtd_nums, subdevs, dev_count);
	if (ret)
		goto err;

	ret = mtd_raid_create(raid_level, subdevs, dev_count, substripe_size);
	if (ret)
		goto err;
	return 0;
err:
	put_subdevs(subdevs, dev_count);
	kfree(subdevs);
	return ret;
}

int mtd_raid_ioctl_destroy(struct mtd_raid_destroy_req *destroy_req)
{
	struct mtd_raid *raid = NULL;
	struct mtd_info **subdevs = NULL;
	int dev_count;
	int ret = 0;

	raid = mtd_raid_list_get(destroy_req->mtd_num);
	if (!raid)
		return -EINVAL;

	subdevs = raid->subdevs;
	dev_count = raid->dev_count;
	ret = mtd_raid_destroy(raid);
	if (ret)
		goto out;

	put_subdevs(subdevs, dev_count);
	kfree(subdevs);
out:
	return ret;
}

static long ctrl_cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *argp = (void __user *)arg;

	if (!capable(CAP_SYS_RESOURCE))
		return -EPERM;

	switch (cmd) {
	case MTD_RAID_IOC_CREATE:
	{
		struct mtd_raid_create_req *req;
		int dev_count = 0;
		int *mtd_nums = NULL;

		req = memdup_user(argp, sizeof(*req));
		if (IS_ERR(req)) {
			err = PTR_ERR(req);
			goto out;
		}

		dev_count = req->dev_count;
		mtd_nums = memdup_user(argp + sizeof(*req), sizeof(int) * dev_count);
		if (IS_ERR(mtd_nums)) {
			err = PTR_ERR(mtd_nums);
			goto out;
		}

		err = mtd_raid_ioctl_create(req, mtd_nums);
		break;
	}
	case MTD_RAID_IOC_DESTROY:
	{
		struct mtd_raid_destroy_req *req;

		req = memdup_user(argp, sizeof(*req));
		if (IS_ERR(req)) {
			err = PTR_ERR(req);
			goto out;
		}

		err = mtd_raid_ioctl_destroy(req);
		break;
	}

	default:
		err = -ENOTTY;
		break;
	}

out:
	return err;
}

#ifdef CONFIG_COMPAT
static long ctrl_cdev_compat_ioctl(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	unsigned long translated_arg = (unsigned long)compat_ptr(arg);

	return ctrl_cdev_ioctl(file, cmd, translated_arg);
}
#else
#define ctrl_cdev_compat_ioctl NULL
#endif

const struct file_operations mtd_raid_ctrl_cdev_operations = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = ctrl_cdev_ioctl,
	.compat_ioctl   = ctrl_cdev_compat_ioctl,
	.llseek		= no_llseek,
};
