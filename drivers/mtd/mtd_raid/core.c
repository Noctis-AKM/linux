/*
 * Part of MTD RAID
 *
 * Copyright (C) 2015 Dongsheng Yang. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Authors: Dongsheng Yang <yangds.fnst@cn.fujitsu.com>
 */

/*
 * TODO
 * 	- merge requests
 * 	- To support writev
 * 	- raid10
 * 	- raid5/6
 *
 * This is the core part of mtd raid layer. We implemented the generic mtd_func
 * in this part. each raid devices share the same generic mtd interfaces. There
 * is only interfaces for each mtd operations, the real io with each flashes are
 * handled by raid_io.c.
 *
 * There is a global list for all struct mtd_raid. It's a 2-D list, we give each
 * raid level a list to manage all mtd_raids in this level. When a mtd raid device
 * created, we need to register it into related list. When a mtd raid device to
 * be destroyed, we need to unregister it from related list.
 **/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <mtd/mtd-raid-user.h>

#include "mtd_raid.h"

/* We have a raid_list for each raid level. */
struct raid_list {
	int count;
	struct list_head head;
};

/* To protect mtd_raid_list. */
static spinlock_t mtd_raid_list_lock;
static struct raid_list mtd_raid_list[MTD_RAID_LEVEL_MAX];

int mtd_raid_list_init(void)
{
	int i = 0;

	spin_lock_init(&mtd_raid_list_lock);
	for (i = 0; i < MTD_RAID_LEVEL_MAX; i++)
		INIT_LIST_HEAD(&mtd_raid_list[i].head);

	return 0;
}

void mtd_raid_list_destroy(void)
{
	struct mtd_raid *mtd_raid, *next;
	int i = 0;

	for (i = 0; i < MTD_RAID_LEVEL_MAX; i++) {
		list_for_each_entry_safe(mtd_raid, next, &mtd_raid_list[i].head, node) {
			mtd_raid_destroy(mtd_raid);
		}
	}
}

int mtd_raid_list_register(enum mtd_raid_level raid_level, struct mtd_raid *mtd_raid)
{
	int raid_id = 0;

	spin_lock(&mtd_raid_list_lock);
	list_add_tail(&mtd_raid->node, &mtd_raid_list[raid_level].head);
	raid_id = ++mtd_raid_list[raid_level].count;
	spin_unlock(&mtd_raid_list_lock);

	return raid_id;
}

struct mtd_raid *mtd_raid_list_get(int mtd_num)
{
	struct mtd_raid *raid;
	int i = 0;

	spin_lock(&mtd_raid_list_lock);
	for (i = 0; i < MTD_RAID_LEVEL_MAX; i++) {
		list_for_each_entry(raid, &mtd_raid_list[i].head, node) {
			if (raid->mtd.index == mtd_num) {
				spin_unlock(&mtd_raid_list_lock);
				return raid;
			}
		}
	}
	spin_unlock(&mtd_raid_list_lock);

	return NULL;
}

void mtd_raid_list_unregister(struct mtd_raid *mtd_raid)
{
	spin_lock(&mtd_raid_list_lock);
	list_del(&mtd_raid->node);
	spin_unlock(&mtd_raid_list_lock);
}

/* MTD interfaces */
/* Check the range of addr:len to see is that out of address space */
int check_offs(struct mtd_info *mtd, loff_t addr, size_t len)
{
        /* Do not allow out of address space of device */
        if (addr < 0) {
                pr_err("%s: From a negative address.\n",
                                        __func__);
                return -EINVAL;
        }

        if (addr > mtd->size) {
                pr_err("%s: From an address out of size.\n",
                                        __func__);
                return -EINVAL;
        }

        if (len > mtd->size - addr) {
                pr_err("%s: Read past size of mtd device.\n",
                                        __func__);
                return -EINVAL;
        }

        return 0;
}

/* It's more strict than check_offs(). It also check the ofs and len
 * to see are they aligned with aligned_size.
 **/
int check_offs_aligned(struct mtd_info *mtd, loff_t ofs, uint64_t len,
                       size_t aligned_size)
{
        int ret = 0;
	loff_t tmp_ofs = ofs;
	loff_t tmp_len = len;

        /* Start address must align on given length */
        if (do_div(tmp_ofs, aligned_size)) {
                pr_err("%s: Unaligned address\n", __func__);
                return -EINVAL;
        }

        /* Length must align on given length */
        if (do_div(tmp_len, aligned_size)) {
                pr_err("%s: Length not aligned\n",
                                        __func__);
                return -EINVAL;
        }

        /* Do not allow out of address space of device */
        ret = check_offs(mtd, ofs, len);

        return ret;
}

/* Interface for mtd->_read() */
int mtd_raid_read(struct mtd_info *mtd, loff_t from, size_t len,
		  size_t *retlen, u_char *buf)
{
	loff_t subdev_off;
	int ret = 0, err = 0;
	int devid, i_copy = 0;
	size_t retsize, size;
	struct mtd_raid_ctx *ctx; 
	struct mtd_raid_dev *subdev;
	struct mtd_raid *raid = MTD_RAID(mtd);
	struct mtd_raid_read_request *read_req;
	struct mtd_raid_request *request;

	if (check_offs(mtd, from, len))
		return -EINVAL;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ret = mtd_raid_ctx_init(ctx);
	if (ret)
		goto free;

	while (len) {
		err = raid->ops->logical_to_physical(raid, from, len, i_copy,
						     &devid, &subdev_off, &size);
		if (err) {
			ret = err;
			goto out;
		}

		subdev = &raid->devs[devid];
		err = mtd_raid_dev_read(ctx, subdev, subdev_off, size, &retsize, buf);
		if (unlikely(err)) {
			ret = err;
			goto out;
		}

		buf += retsize;
		from += retsize;
		len -= retsize;
	}
wait:
	ret = mtd_raid_ctx_wait(ctx);
	if (ret) {
		if (!list_empty(&ctx->error_list)) {
			request = list_first_entry(&ctx->error_list,
							struct mtd_raid_request, node);
			read_req = READ_REQUEST(request);
			ret = read_req->retval;
			goto out;
		} else if (!list_empty(&ctx->failed_list)) {
			ret =  -EBADMSG;
		} else {
			if (!ret)
				ret = -EUCLEAN;
		}
	} else {
		goto out;
	}
	
	if (++i_copy >= raid->ncopies)
		goto out;

	ret = mtd_raid_ctx_retry(ctx, i_copy);
	if (ret)
		goto out;
	goto wait;
out:
	mtd->ecc_stats.failed += ctx->failed;
	mtd->ecc_stats.corrected += ctx->corrected;
	/* Fill retlen */
	*retlen = 0;
	list_for_each_entry(request, &ctx->all_list, node_all) {
		read_req = READ_REQUEST(request);
		if (read_req->retval && !mtd_is_bitflip_or_eccerr(read_req->retval))
			break;
		*retlen += read_req->retlen;
	}
	mtd_raid_ctx_destroy(ctx);
free:
	kfree(ctx);

	return ret;
}

/* Interface for mtd->_write() */
int mtd_raid_write(struct mtd_info *mtd, loff_t to, size_t len,
		   size_t * retlen, const u_char * buf)
{
	int err = 0;
	int i = 0;
	int devid;
	loff_t subdev_off;
	size_t retsize, size;
	struct mtd_info *subdev = NULL;
	struct mtd_raid *raid = MTD_RAID(mtd);

	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;

	if (check_offs(mtd, to, len))
		return -EINVAL;

	while (len) {
		for (i = 0; i < raid->ncopies; i++) {
			err = raid->ops->logical_to_physical(raid, to, len, i,
							     &devid, &subdev_off, &size);
			if (err)
				goto out;

			subdev = raid->devs[devid].mtd;
			err = mtd_write(subdev, subdev_off, size, &retsize, buf);
			if (unlikely(err))
				goto out;
		}

		*retlen += retsize;
		len -= retsize;
		buf += retsize;
		to += retsize;
	}

out:
	return err;
}

int mtd_raid_read_oob(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops)
{
	size_t size, readlen = ops->len;
	loff_t subdev_off;
	struct mtd_oob_ops devops = *ops;
	uint64_t oobsize;
	struct mtd_raid_ctx *ctx; 
	struct mtd_raid_dev *subdev;
	struct mtd_raid *raid = MTD_RAID(mtd);
	struct mtd_raid_read_oob_request *read_oob_req;
	struct mtd_raid_request *request;
	int i_copy = 0, devid = 0, i = 0, ret = 0;

	/* Check parameters for reading oob */
	if (ops->datbuf && check_offs(mtd, from, ops->len))
		return -EINVAL;

	/* Get oobsize depending on mode */
	if (ops->mode == MTD_OPS_AUTO_OOB)
		oobsize = mtd->oobavail;
	else
		oobsize = mtd->oobsize;

	/* Check ooboffs */
	if (devops.ooboffs >= oobsize)
		return -EINVAL;

	/* Check len and from */
	oobsize -= devops.ooboffs;
	if (devops.datbuf) {
		readlen = devops.len;
	} else {
		readlen = DIV_ROUND_UP(devops.ooblen, oobsize);
		readlen *= mtd->writesize;
	}

	if (readlen > mtd->size - from)
		return -EINVAL;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ret = mtd_raid_ctx_init(ctx);
	if (ret)
		goto free;

	while (readlen > 0) {
		ret = raid->ops->logical_to_physical(raid, from, readlen, i,
						     &devid, &subdev_off, &size);
		if (unlikely(ret))
			goto out;

		if (devops.datbuf) {
			devops.len = size;
		} else {
			if (devops.ooblen > (size / mtd->writesize * oobsize))
				devops.ooblen = size / mtd->writesize * oobsize;
		}

		/* Read data from subdev */
		subdev = &raid->devs[devid];
		ret = mtd_raid_dev_read_oob(ctx, subdev, subdev_off, size, &devops);
		if (unlikely(ret))
			goto out;

		readlen -= size;
		from += size;

		if (devops.datbuf)
			devops.datbuf += devops.len;

		if (devops.oobbuf)
			devops.oobbuf += devops.ooblen;
	}

wait:
	ret = mtd_raid_ctx_wait(ctx);
	if (ret) {
		/* Not all request succeeded */
		if (!list_empty(&ctx->error_list)) {
			request = list_first_entry(&ctx->error_list,
						   struct mtd_raid_request, node);
			read_oob_req = READ_OOB_REQUEST(request);
			ret = read_oob_req->retval;
			goto out;
		} else if (!list_empty(&ctx->failed_list)) {
			ret =  -EBADMSG;
		} else {
			if (!ret)
				ret = -EUCLEAN;
		}
	} else {
		goto out;
	}
	
	if (++i_copy >= raid->ncopies)
		goto out;

	ret = mtd_raid_ctx_retry(ctx, i_copy);
	if (ret)
		goto out;
	goto wait;
out:
	mtd->ecc_stats.failed += ctx->failed;
	mtd->ecc_stats.corrected += ctx->corrected;
	/* Fill retlen */
	ops->retlen = ops->oobretlen = 0;
	list_for_each_entry(request, &ctx->all_list, node_all) {
		read_oob_req = READ_OOB_REQUEST(request);
		if (read_oob_req->retval && !mtd_is_bitflip_or_eccerr(read_oob_req->retval))
			break;
		ops->retlen += read_oob_req->ops.retlen;
		ops->oobretlen += read_oob_req->ops.oobretlen;
	}
	mtd_raid_ctx_destroy(ctx);
free:
	kfree(ctx);

	return ret;
}

int mtd_raid_write_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	struct mtd_oob_ops devops = *ops;
	uint64_t oobsize;
	int devid = 0, i = 0, ret = 0;
	loff_t subdev_off;
	size_t size, writelen = ops->len;
	struct mtd_info *subdev = NULL;

	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;

	if (ops->datbuf && check_offs(mtd, to, ops->len))
		return -EINVAL;

	ops->retlen = ops->oobretlen = 0;

	if (ops->mode == MTD_OPS_AUTO_OOB)
		oobsize = mtd->oobavail;
	else
		oobsize = mtd->oobsize;

	if (to < 0 || to > mtd->size)
		return -EINVAL;

	if (devops.ooboffs >= oobsize)
		return -EINVAL;

	oobsize -= devops.ooboffs;

	if (devops.datbuf) {
		writelen = devops.len;
	} else {
		writelen = DIV_ROUND_UP(devops.ooblen, oobsize);
		writelen *= mtd->writesize;
	}

	if (writelen > mtd->size - to)
		return -EINVAL;

	while (writelen > 0) {
		for (i = 0; i < raid->ncopies; i++) {
			ret = raid->ops->logical_to_physical(raid, to, writelen, i,
							     &devid, &subdev_off, &size);
			if (unlikely(ret))
				goto out;

			if (devops.datbuf) {
				devops.len = size;
			} else {
				if (devops.ooblen > (size / mtd->writesize * oobsize))
					devops.ooblen = size / mtd->writesize * oobsize;
			}

			subdev = raid->devs[devid].mtd;
			ret = mtd_write_oob(subdev, subdev_off, &devops);
			if (ret)
				goto out;

			writelen -= size;
			to += size;

			if (devops.datbuf) {
				devops.datbuf += devops.len;
				ops->retlen += devops.len;
			}

			if (devops.oobbuf) {
				devops.oobbuf += devops.ooblen;
				ops->oobretlen += devops.ooblen;
			}
		}
	}
out:
	return ret;
}

//TODO make it async and paralleled
/* lblock means logical block */
static int raid_erase_lblock(struct mtd_raid *raid, struct erase_info *instr)
{
	int devid;
	size_t size;
	loff_t subdev_off;
	int i, icopy = 0;
	int err = 0, ret = 0;
	uint64_t addr, len;
	struct mtd_raid_dev *subdev = NULL;

	addr = instr->addr;
	len = 0;
	for (i = 0; i < raid->npebs_per_leb; i++) {
		for (icopy = 0; icopy < raid->ncopies; icopy++) {
			err = raid->ops->logical_to_physical(raid, addr, len, icopy, &devid, &subdev_off, &size);
			if (err)
				goto out;

			subdev = &raid->devs[devid];
			if (!(subdev->mtd->flags & MTD_WRITEABLE)) {
				ret = -EROFS;
				goto out;
			}

			instr->addr = subdev_off;

			ret = mtd_raid_dev_erase(subdev, instr);
			if (ret)
				goto out;

			if (instr->state != MTD_ERASE_DONE) {
				ret = -EIO;
				goto out;
			}
		}

		addr += raid->substripe_size;
	}

out:
	return ret;
}

int mtd_raid_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	struct erase_info *erase;
	loff_t logical;
	loff_t length;
	int err = 0;

	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;

	if (check_offs_aligned(mtd, instr->addr, instr->len, mtd->erasesize))
		return -EINVAL;

	instr->fail_addr = MTD_FAIL_ADDR_UNKNOWN;

	erase = kmalloc(sizeof(struct erase_info), GFP_KERNEL);
	if (!erase)
		return -ENOMEM;

	*erase = *instr;
	logical = instr->addr;
	length = instr->len;

	err = 0;
	while (length > 0) {
		erase->addr = logical;
		err = raid_erase_lblock(raid, erase);
		if (err)
			break;

		logical += raid->mtd.erasesize;
		length -= raid->mtd.erasesize;
	}

	instr->state = erase->state;
	kfree(erase);

	if (err)
		return err;

	if (instr->callback)
		instr->callback(instr);

	return 0;

}

void mtd_raid_sync(struct mtd_info *mtd)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	int i;

	for (i = 0; i < raid->dev_count; i++) {
		struct mtd_info *subdev = raid->devs[i].mtd;
		mtd_sync(subdev);
	}
}

int mtd_raid_suspend(struct mtd_info *mtd)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	int i, err = 0;

	for (i = 0; i < raid->dev_count; i++) {
		struct mtd_info *subdev = raid->devs[i].mtd;
		if ((err = mtd_suspend(subdev)) < 0)
			return err;
	}
	return err;
}

void mtd_raid_resume(struct mtd_info *mtd)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	int i;

	for (i = 0; i < raid->dev_count; i++) {
		struct mtd_info *subdev = raid->devs[i].mtd;
		mtd_resume(subdev);
	}
}

int mtd_raid_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	int ret = 0, err = 0, i = 0;
	int devid;
	loff_t subdev_off;
	size_t size;
	struct mtd_info *subdev = NULL;

	while (len) {
		for (i = 0; i < raid->ncopies; i++) {
			err = raid->ops->logical_to_physical(raid, ofs, len, i, &devid, &subdev_off, &size);
			if (err) {
				ret = err;
				goto out;
			}

			subdev = raid->devs[devid].mtd;
			err = mtd_lock(subdev, subdev_off, size);
			if (unlikely(err)) {
				ret = err;
				goto out;
			}
		}

		len -= size;
		ofs += size;
	}
out:
	return ret;
}

int mtd_raid_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	int ret = 0, err = 0, i = 0;
	int devid;
	loff_t subdev_off;
	size_t size;
	struct mtd_info *subdev = NULL;

	while (len) {
		for (i = 0; i < raid->ncopies; i++) {
			err = raid->ops->logical_to_physical(raid, ofs, len, i, &devid, &subdev_off, &size);
			if (err) {
				ret = err;
				goto out;
			}

			subdev = raid->devs[devid].mtd;
			err = mtd_unlock(subdev, subdev_off, size);
			if (unlikely(err)) {
				ret = err;
				goto out;
			}
		}

		len -= size;
		ofs += size;
	}

out:
	return ret;
}

int mtd_raid_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	int i = 0;
	loff_t from = ofs, subdev_off;
	size_t len;
	int devid;
	size_t size;
	struct mtd_info *subdev = NULL;

	if (!mtd_can_have_bb(raid->devs[0].mtd))
		return 0;

	ofs -= do_div(from, mtd->erasesize);
	from = ofs;
	len = mtd->erasesize;
	while (len) {
		for (i = 0; i < raid->ncopies; i++) {
			raid->ops->logical_to_physical(raid, from, len, i, &devid, &subdev_off, &size);

			subdev = raid->devs[devid].mtd;
			if (mtd_block_isbad(subdev, subdev_off))
				return 1;
		}

		len -= size;
		from += size;
	}

	return 0;
}

int mtd_raid_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct mtd_raid *raid = MTD_RAID(mtd);
	int i, err = 0;
	loff_t from = ofs;
	size_t len;
	int devid;
	loff_t subdev_off;
	size_t size;
	struct mtd_info *subdev = NULL;

	ofs -= do_div(from, mtd->erasesize);
	from = ofs;
	len = mtd->erasesize;
	while (len) {
		for (i = 0; i < raid->ncopies; i++) {
			err = raid->ops->logical_to_physical(raid, from, len, i, &devid, &subdev_off, &size);
			if (err)
				goto out;

			subdev = raid->devs[devid].mtd;
			err = mtd_block_markbad(subdev, subdev_off);
			if (err)
				goto out;
			else
				mtd->ecc_stats.badblocks++;
		}

		len -= size;
		from += size;
	}

out:
	return err;
}

int mtd_raid_init(struct mtd_raid *raid, int *mtd_nums, int dev_count, size_t substripe_size)
{
	struct mtd_info *subdev = NULL;
	struct mtd_info *mtd = NULL;
	int ret = 0;
	int i = 0;

	INIT_LIST_HEAD(&raid->node);
	raid->substripe_size = substripe_size;
	raid->dev_count = dev_count;

	for (i = 0; i < dev_count; i++) {
		subdev = get_mtd_device(NULL, mtd_nums[i]);
		if (IS_ERR(subdev)) {
			ret = PTR_ERR(subdev);
			pr_err("error: Cannot get MTD device. mtd_num: %d\n", mtd_nums[i]);
			goto out;
		}
		raid->devs[i].mtd = subdev;
		raid->devs[i].id = i;
	}

	mtd = &raid->mtd;
	subdev = raid->devs[0].mtd;

	if (raid->substripe_size == 0)
		raid->substripe_size = subdev->writesize;

	mtd->owner = THIS_MODULE;
	mtd->type = subdev->type;
	mtd->flags = subdev->flags;
	mtd->writesize = subdev->writesize;
	mtd->writebufsize = subdev->writebufsize;
	mtd->subpage_sft = subdev->subpage_sft;
	mtd->oobsize = subdev->oobsize;
	mtd->oobavail = subdev->oobavail;
	mtd->ecclayout = subdev->ecclayout;

	mtd->_erase = mtd_raid_erase;
	mtd->_read = mtd_raid_read;
	mtd->_write = mtd_raid_write;
	mtd->_sync = mtd_raid_sync;
	mtd->_lock = mtd_raid_lock;
	mtd->_unlock = mtd_raid_unlock;
	mtd->_suspend = mtd_raid_suspend;
	mtd->_resume = mtd_raid_resume;

	if (subdev->_read_oob)
		mtd->_read_oob = mtd_raid_read_oob;
	if (subdev->_write_oob)
		mtd->_write_oob = mtd_raid_write_oob;
	if (subdev->_block_isbad)
		mtd->_block_isbad = mtd_raid_block_isbad;
	if (subdev->_block_markbad)
		mtd->_block_markbad = mtd_raid_block_markbad;

	for (i = 1; i < dev_count; i++) {
		if (mtd->flags != raid->devs[i].mtd->flags) {
			/*
			 * Expect all flags except MTD_WRITEABLE to be
			 * equal on all subdevices.
			 */
			if ((mtd->flags ^ raid->devs[i].mtd->
			     flags) & ~MTD_WRITEABLE) {
				printk("Incompatible device flags on \"%s\"\n",
				       raid->devs[i].mtd->name);
				ret = -EINVAL;
				goto out;
			} else {
				/* if writeable attribute differs,
				   make super device writeable */
				mtd->flags |=
				    raid->devs[i].mtd->flags & MTD_WRITEABLE;
			}
		}

		if (mtd->writesize   !=  raid->devs[i].mtd->writesize ||
		    mtd->subpage_sft != raid->devs[i].mtd->subpage_sft ||
		    mtd->oobsize    !=  raid->devs[i].mtd->oobsize ||
		    !mtd->_read_oob  != !raid->devs[i].mtd->_read_oob ||
		    !mtd->_write_oob != !raid->devs[i].mtd->_write_oob) {
			printk("Incompatible OOB or ECC data on \"%s\"\n",
			       raid->devs[i].mtd->name);
			ret = -EINVAL;
			goto out;
		}

		if (mtd->writebufsize != raid->devs[i].mtd->writebufsize) {
			pr_err("Incompatible writebufsize on \"%s\"",
			       raid->devs[i].mtd->name);
			ret = -EINVAL;
			goto out;
		}
	}

	if (raid->ops->init) {
		ret = raid->ops->init(raid, dev_count, substripe_size);
		if (ret)
			goto out;
	}

	for (i = 0; i < dev_count; i++) {
		/*
		 * Init bg thread for each raid_dev to handle io requests.
		 */
		INIT_LIST_HEAD(&raid->devs[i].list);
		raid->devs[i].thread = kthread_create(mtd_raid_dev_thread, &raid->devs[i],
						      "%s_thread_%d", raid->name, i);
	}

	return 0;
out:
	return ret;
}

int mtd_raid_destroy(struct mtd_raid *raid);

int mtd_raid_create(enum mtd_raid_level raid_level, int *mtd_nums, int dev_count, int substripe_size)
{
	int ret = 0;
	struct mtd_raid *raid = NULL;

	switch (raid_level){
	case MTD_RAID_LEVEL_SINGLE:
	{
		raid = mtd_raid_single_create(mtd_nums, dev_count, substripe_size);
		if (!raid) {
			pr_err("MTD RAID: Failed to create raid single device.");
			ret = -EINVAL;
			goto out;
		}
		break;	
	}
	case MTD_RAID_LEVEL_RAID0:
	{
		raid = mtd_raid0_create(mtd_nums, dev_count, substripe_size);
                if (!raid) {
                        pr_err("MTD RAID: Failed to create raid0 device.");
                        ret = -EINVAL;
                        goto out;
                }
		break;
	}
	case MTD_RAID_LEVEL_RAID1:
	{
		raid = mtd_raid1_create(mtd_nums, dev_count, substripe_size);
                if (!raid) {
                        pr_err("MTD RAID: Failed to create raid1 device.");
                        ret = -EINVAL;
                        goto out;
                }
		break;
	}
	default:
		pr_err("MTD RAID: Unsupported raid level: %d.", raid_level);
		ret = -ENOTSUPP;
		goto out;
	}

	ret = mtd_raid_init(raid, mtd_nums, dev_count, substripe_size);
	if (ret)
		goto destroy;

	return mtd_device_register(&raid->mtd, NULL, 0);

destroy:
	mtd_raid_destroy(raid);
out:
	return ret;
}

int mtd_raid_destroy(struct mtd_raid *raid)
{
	int i = 0;
	int ret = 0;

	ret = mtd_device_unregister(&raid->mtd);
	if (ret)
		goto out;

	for (i = 0; i < raid->dev_count; i++) {
		if (raid->devs[i].mtd)
			put_mtd_device(raid->devs[i].mtd);

		if (raid->devs[i].thread)
			kthread_stop(raid->devs[i].thread);
	}

	if (raid->ops->destroy)
		raid->ops->destroy(raid);
out:
	return ret;
}

static struct miscdevice mtd_raid_ctrl_cdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mtd_raid_ctrl",
	.fops = &mtd_raid_ctrl_cdev_operations,
};

static int __init init_mtd_raid(void)
{
	int err = 0;

	err = mtd_raid_list_init();
	if (err)
		goto out;

	err = misc_register(&mtd_raid_ctrl_cdev);
	if (err) {
		pr_err("MTD RAID error: cannot register device");
		goto out;
	}

	return 0;
out:
	pr_err("MTD RAID error: cannot initialize MTD RAID, error %d", err);
	return err;
}

static void __exit cleanup_mtd_raid(void)
{
	misc_deregister(&mtd_raid_ctrl_cdev);
	mtd_raid_list_destroy();
}

module_init(init_mtd_raid);
module_exit(cleanup_mtd_raid);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dongsheng Yang <yangds.fnst@cn.fujitsu.com>");
MODULE_DESCRIPTION("Support for MTD RAID");
