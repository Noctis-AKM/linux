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
 * This file handles the all io-related work.
 */

#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/slab.h>

#include "mtd_raid.h"

/**
 * Context related operations:
 * 	mtd_raid_ctx_init()	--> Init a ctx
 * 				--> Attach requests to ctx
 * 	mtd_raid_ctx_wait()	--> Wait all requests end
 * 	mtd_raid_ctx_retry()	--> Retry failed requests
 * 	mtd_raid_ctx_destroy()	--> Destroy the ctx
 */
int mtd_raid_ctx_init(struct mtd_raid_ctx* ctx)
{
	spin_lock_init(&ctx->lock);
	INIT_LIST_HEAD(&ctx->all_list);
	INIT_LIST_HEAD(&ctx->submit_list);
	INIT_LIST_HEAD(&ctx->complete_list);
	INIT_LIST_HEAD(&ctx->failed_list);
	INIT_LIST_HEAD(&ctx->corrected_list);
	INIT_LIST_HEAD(&ctx->error_list);

	ctx->failed = ctx->corrected = ctx->errored = 0;
	ctx->wait = current;

	return 0;
}

int mtd_raid_ctx_wait(struct mtd_raid_ctx *ctx)
{
	int ret = 0;

	while (1) {
		spin_lock(&ctx->lock);
		if (list_empty(&ctx->submit_list)) {
			ret = ctx->failed + ctx->corrected + ctx->errored;
			spin_unlock(&ctx->lock);
			return ret;
		}

		spin_unlock(&ctx->lock);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		set_current_state(TASK_RUNNING);

		cond_resched();
	}
}

void mtd_raid_ctx_destroy(struct mtd_raid_ctx *ctx)
{
	struct mtd_raid_request *request, *next;

	list_for_each_entry_safe(request, next, &ctx->all_list, node_all) {
		if (request->end_func)
			request->end_func(request);
	}
}

int mtd_raid_ctx_retry(struct mtd_raid_ctx *ctx, int i_copy)
{
	struct mtd_raid_request *request;
	int ret = 0;

	list_for_each_entry(request, &ctx->failed_list, node) {
		ret = request->retry_func(request, i_copy);
		if (ret)
			goto out;
		spin_lock(&ctx->lock);
		list_move_tail(&request->node, &ctx->submit_list);
		spin_unlock(&ctx->lock);
	}
out:
	return ret;
}

/*
 * hooks for each type of request
 * */
static void read_req_func(struct mtd_raid_request *request)
{
	struct mtd_raid_read_request *read_req;
	struct mtd_raid_ctx *ctx;
	struct mtd_raid_dev *raid_dev;
	int ret = 0;

	read_req = READ_REQUEST(request);
	raid_dev = request->raid_dev;
	ret = mtd_read(raid_dev->mtd, read_req->from, read_req->len,
		       &read_req->retlen, read_req->buf);

	read_req->retval = ret;
	ctx = request->ctx;
	spin_lock(&ctx->lock);
	list_del_init(&request->node);
	if (unlikely(ret)) {
		if (mtd_is_eccerr(ret)) {
			ctx->failed++;
			list_add_tail(&request->node, &ctx->failed_list);
		} else if (mtd_is_bitflip(ret)) {
			ctx->corrected++;
			list_add_tail(&request->node, &ctx->corrected_list);
		} else {
			ctx->errored++;
			list_add_tail(&request->node, &ctx->error_list);
		}
	} else {
		list_add_tail(&request->node, &ctx->complete_list);
	}
	spin_unlock(&ctx->lock);
	wake_up_process(ctx->wait);

	return;
}

static void read_oob_req_func(struct mtd_raid_request *request)
{
	struct mtd_raid_read_oob_request *read_oob_req;
	struct mtd_raid_ctx *ctx;
	struct mtd_raid_dev *raid_dev;
	int ret = 0;

	read_oob_req = READ_OOB_REQUEST(request);
	raid_dev = request->raid_dev;
	ret = mtd_read_oob(raid_dev->mtd, read_oob_req->from, &read_oob_req->ops);

	read_oob_req->retval = ret;
	ctx = request->ctx;
	spin_lock(&ctx->lock);
	list_del_init(&request->node);
	if (unlikely(ret)) {
		if (mtd_is_eccerr(ret)) {
			ctx->failed++;
			list_add_tail(&request->node, &ctx->failed_list);
		} else if (mtd_is_bitflip(ret)) {
			ctx->corrected++;
			list_add_tail(&request->node, &ctx->corrected_list);
		} else {
			ctx->errored++;
			list_add_tail(&request->node, &ctx->error_list);
		}
	} else {
		list_add_tail(&request->node, &ctx->complete_list);
	}
	spin_unlock(&ctx->lock);
	wake_up_process(ctx->wait);

	return;
}

static int read_req_retry_func(struct mtd_raid_request *request, int i_copy)
{
	struct mtd_raid *mtd_raid;
	struct mtd_raid_read_request *read_req;
	loff_t address, subdev_off;
	size_t length, size;
	int devid, ret = 0;

	mtd_raid = request->raid_dev->raid;
	read_req = READ_REQUEST(request);
	if (!mtd_raid->ops->physical_to_logical || !mtd_raid->ops->logical_to_physical)
		return -EINVAL;

	subdev_off = read_req->from;
	size = read_req->len;
	devid = request->raid_dev->id;
	ret = mtd_raid->ops->physical_to_logical(mtd_raid, devid, subdev_off, size,
						 &address, &length);
	if (ret)
		goto out;

	ret = mtd_raid->ops->logical_to_physical(mtd_raid, address, length, i_copy,
						 &devid, &subdev_off, &size);
	if (ret)
		goto out;

	/* Fill request with the address of new copy */
	request->raid_dev = &mtd_raid->devs[devid];
	read_req->from = subdev_off;
	read_req->len = size;
out:
	return ret;
}

static int read_oob_req_retry_func(struct mtd_raid_request *request, int i_copy)
{
	struct mtd_raid *mtd_raid;
	struct mtd_raid_read_oob_request *read_oob_req;
	loff_t address, subdev_off;
	size_t length, size;
	int devid, ret = 0;

	mtd_raid = request->raid_dev->raid;
	read_oob_req = READ_OOB_REQUEST(request);
	if (!mtd_raid->ops->physical_to_logical || !mtd_raid->ops->logical_to_physical)
		return -EINVAL;

	subdev_off = read_oob_req->from;
	size = read_oob_req->len;
	devid = request->raid_dev->id;
	ret = mtd_raid->ops->physical_to_logical(mtd_raid, devid, subdev_off, size,
						 &address, &length);
	if (ret)
		goto out;

	ret = mtd_raid->ops->logical_to_physical(mtd_raid, address, length, i_copy,
						 &devid, &subdev_off, &size);
	if (ret)
		goto out;

	/* Fill request with the address of new copy */
	request->raid_dev = &mtd_raid->devs[devid];
	read_oob_req->from = subdev_off;
	read_oob_req->len = size;
out:
	return ret;
}

/* Generic end_func for request */
static void request_end_func(struct mtd_raid_request *request)
{
	struct mtd_raid_ctx *ctx = NULL;
	struct mtd_raid_dev *raid_dev = NULL;

	ctx = request->ctx;
	spin_lock(&ctx->lock);
	list_del(&request->node);
	list_del(&request->node_all);
	spin_unlock(&ctx->lock);

	raid_dev = request->raid_dev;
	spin_lock(&raid_dev->lock);
	list_del(&request->node_request);
	spin_unlock(&raid_dev->lock);
}

static void read_req_end_func(struct mtd_raid_request *request)
{
	struct mtd_raid_read_request *read_req;

	read_req = READ_REQUEST(request);
	request_end_func(request);
	kfree(read_req);
}

static void read_oob_req_end_func(struct mtd_raid_request *request)
{
	struct mtd_raid_read_oob_request *read_oob_req;

	read_oob_req = READ_OOB_REQUEST(request);
	request_end_func(request);
	kfree(read_oob_req);
}

/**
 * Thread for each raid_dev.
 *
 * It get requests from raid_dev->list and do the
 * requested work, until raid_dev to be empty. Then
 * go to sleep.
 */
int mtd_raid_dev_thread(void *u)
{
	struct mtd_raid_dev *raid_dev = u;
	struct mtd_raid_request *request;

	set_freezable();
	for (;;) {
		if (kthread_should_stop())
			break;

		if (try_to_freeze())
			continue;

		spin_lock(&raid_dev->lock);
		if (list_empty(&raid_dev->list)) {
			set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock(&raid_dev->lock);
			schedule();
			continue;
		}
		/*
		 * Get the first request from request list.
		 **/
		request = list_first_entry(&raid_dev->list,
						struct mtd_raid_request, node_request);
		list_del_init(&request->node_request);
		spin_unlock(&raid_dev->lock);

		if (request->func)
			request->func(request);
		cond_resched();
	}

	return 0;
}

/* Interfaces of raid_dev */

/*
 * read interface for raid_dev.
 * */
int mtd_raid_dev_read(struct mtd_raid_ctx *ctx, struct mtd_raid_dev *raid_dev,
		      loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	struct mtd_raid_read_request *read_req;
	struct mtd_raid_request *request;
	int ret = 0;

	/* Alloc a request */
	read_req = kzalloc(sizeof(*read_req), GFP_KERNEL);
	if (!read_req)
		goto out;

	request = &read_req->request;
	/* Init the request */
	INIT_LIST_HEAD(&request->node);
	INIT_LIST_HEAD(&request->node_all);
	INIT_LIST_HEAD(&request->node_request);

	request->ctx = ctx;
	request->raid_dev = raid_dev;
	request->type = MTD_RAID_REQ_READ;
	request->func = read_req_func;
	request->retry_func = read_req_retry_func;
	request->end_func = read_req_end_func;

	/* Init read_request */
	read_req->from = from;
	read_req->len = len;
	read_req->buf = buf;

	/* Add request to context */
	spin_lock(&ctx->lock);
	list_add_tail(&request->node, &ctx->submit_list);
	list_add_tail(&request->node_all, &ctx->all_list);
	spin_unlock(&ctx->lock);

	/* Dispatch request to related raid_dev */
	spin_lock(&raid_dev->lock);
	list_add_tail(&request->node_request, &raid_dev->list);
	spin_unlock(&raid_dev->lock);

	/* Wakeup background thread to handle requests */
	wake_up_process(raid_dev->thread);

	*retlen = len;
out:
	return ret;
}

/*
 * read_oob interface for raid_dev.
 */
int mtd_raid_dev_read_oob(struct mtd_raid_ctx *ctx, struct mtd_raid_dev *raid_dev,
			  loff_t from, size_t len, struct mtd_oob_ops *ops)
{
	struct mtd_raid_read_oob_request *read_oob_req;
	struct mtd_raid_request *request;
	int ret = 0;

	/* Alloc a request */
	read_oob_req = kzalloc(sizeof(*read_oob_req), GFP_KERNEL);
	if (!read_oob_req)
		goto out;

	request = &read_oob_req->request;
	/* Init the request */
	INIT_LIST_HEAD(&request->node);
	INIT_LIST_HEAD(&request->node_all);
	INIT_LIST_HEAD(&request->node_request);

	request->ctx = ctx;
	request->raid_dev = raid_dev;
	request->type = MTD_RAID_REQ_READ_OOB;
	request->func = read_oob_req_func;
	request->retry_func = read_oob_req_retry_func;
	request->end_func = read_oob_req_end_func;

	/* Init read_request */
	read_oob_req->from = from;
	read_oob_req->len = len;
	memcpy(&read_oob_req->ops, ops, sizeof(*ops));

	/* Add request to context */
	spin_lock(&ctx->lock);
	list_add_tail(&request->node, &ctx->submit_list);
	list_add_tail(&request->node_all, &ctx->all_list);
	spin_unlock(&ctx->lock);

	/* Dispatch request to related raid_dev */
	spin_lock(&raid_dev->lock);
	list_add_tail(&request->node_request, &raid_dev->list);
	spin_unlock(&raid_dev->lock);

	/* Wakeup background thread to handle requests */
	wake_up_process(raid_dev->thread);
out:
	return ret;
}

/*
 * erase interface for raid_dev.
 * */
int mtd_raid_dev_erase(struct mtd_raid_dev *raid_dev, struct erase_info *erase)
{
	int err = 0;
	struct mtd_info *mtd = raid_dev->mtd;

	erase->mtd = mtd;
	erase->len = mtd->erasesize;
	err = mtd_erase(mtd, erase);
	if (err)
		goto out;

	if (erase->state != MTD_ERASE_DONE) {
		err = -EIO;
		goto out;
	}
out:
	return err;
}
