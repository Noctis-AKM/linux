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

#ifndef __MTD_RAID_H
#define __MTD_RAID_H

#include <linux/mtd/mtd.h>
#include <mtd/mtd-abi.h>
#include <mtd/mtd-raid-user.h>

struct mtd_raid;

/*
 * Operations for different mtd raid structure, it
 * differs in different raid level.
 **/
struct mtd_raid_operations {
	int (*init)(struct mtd_raid *, int, size_t);
	void (*destroy)(struct mtd_raid *);

	/* logical_to_physical is the core function in mtd raid,
	 * when we get a logical address from user,
	 * we will call ->logical_to_physical() to get the phisical
	 * devid, address and length.*/
	int (*logical_to_physical)(struct mtd_raid *, loff_t, size_t, int, int *, loff_t *, size_t *);
	int (*physical_to_logical)(struct mtd_raid *, int, loff_t, size_t, loff_t *, size_t *);
};

/*
 * struct mtd_raid_dev, one mtd_raid_dev represent a mtd device
 * int the raid array.
 **/
struct mtd_raid_dev {
	int id;
	struct mtd_raid *raid;
	struct mtd_info *mtd;

	spinlock_t lock;
	struct list_head list;
	struct task_struct *thread;
};

/*
 * Context of a IO, we will send lots of io request to
 * mtd_raid_dev, and mtd_raid_dev will do it in thread.
 * But in order to track the status of each request, we
 * will register them in a mtd_raid_ctx.
 */
struct mtd_raid_ctx {
	spinlock_t lock;
	struct list_head all_list;

	struct list_head submit_list;
	struct list_head complete_list;
	struct list_head failed_list;
	struct list_head corrected_list;
	struct list_head error_list;

	unsigned int failed;
	unsigned int corrected;
	unsigned int errored;

	struct task_struct *wait;
};


/*
 * Type for each io request;
 */
enum mtd_raid_request_type {
	MTD_RAID_REQ_READ	= 0,
	MTD_RAID_REQ_READ_OOB
};

struct mtd_raid_request;
typedef void request_func_t(struct mtd_raid_request *);
typedef void end_func_t(struct mtd_raid_request *);
typedef int retry_func_t(struct mtd_raid_request *request, int i_copy);

struct mtd_raid_request {
	enum mtd_raid_request_type type;
	struct mtd_raid_ctx *ctx;
	struct mtd_raid_dev *raid_dev;

	struct list_head node;
	struct list_head node_all;
	struct list_head node_request;

	/* Main action of this request. */
	request_func_t	*func;
	/*
	 * If this request failed, how to retry. NULL means
	 * don't retry for this request.
	 */
	retry_func_t	*retry_func;
	/*
	 * When we want to destroy this request, what we need
	 * to do.
	 */
	end_func_t	*end_func;
};

/*
 * Request for mtd_read.
 */
struct mtd_raid_read_request {
	struct mtd_raid_request request;

	loff_t from;
	size_t len;
	size_t retlen;
	u_char *buf;
	int retval;
};

/*
 * Request for mtd_read_oob
 */
struct mtd_raid_read_oob_request {
	struct mtd_raid_request request;

	loff_t from;
	size_t len;
	struct mtd_oob_ops ops;
	int retval;
};

/*
 * structure to represent a RAID device
 **/
struct mtd_raid {
	char name[32];
	int ncopies;
	int dev_count;
	int npebs_per_leb;
	int substripe_size;
	/*
	 * This is the "superblock" for this RAID device.
	 * We will fill up it and register it.
	 **/
	struct mtd_info mtd;
	struct list_head node;
	const struct mtd_raid_operations *ops;
	enum mtd_raid_level raid_level;
	struct mtd_raid_dev devs[0];
};

struct mtd_raid_single {
	/* 
	 * Please make the raid to be the last member,
	 * because we will alloc devs appending this structure.
	 */
	struct mtd_raid raid;
};

struct mtd_raid0 {
	// XXX Add reada support here.

	/* 
	 * Please make the raid to be the last member,
	 * because we will alloc devs appending this structure.
	 */
	struct mtd_raid raid;
};

struct mtd_raid1 {
	/*
	 * Please make the raid to be the last member,
	 * because we will alloc devs appending this structure.
	 */
	struct mtd_raid raid;
};

/* Macros to get specified request pointers from generic request */
#define READ_REQUEST(req)				\
	container_of(req, struct mtd_raid_read_request, request)

#define READ_OOB_REQUEST(req)				\
	container_of(req, struct mtd_raid_read_oob_request, request)

/* Macros to get specified mtd_raid pointers from mtd_info pointer */
#define MTD_RAID(mtd)					\
	container_of(mtd, struct mtd_raid, mtd)

#define MTD_RAID_SINGLE(mtd_raid)				\
	container_of(mtd_raid, struct mtd_raid_single, raid)

#define MTD_RAID_RAID0(mtd_raid)				\
	container_of(mtd_raid, struct mtd_raid0, raid)

#define MTD_RAID_RAID1(mtd_raid)				\
	container_of(mtd_raid, struct mtd_raid1, raid)

/* ioctl.c */
extern const struct file_operations mtd_raid_ctrl_cdev_operations;

/* core.c */
int mtd_raid_list_init(void);
void mtd_raid_list_destroy(void);
int mtd_raid_list_register(enum mtd_raid_level raid_level, struct mtd_raid *mtd_raid);
struct mtd_raid *mtd_raid_list_get(int mtd_num);
void mtd_raid_list_unregister(struct mtd_raid *mtd_raid);

int mtd_raid_create(enum mtd_raid_level raid_level, int *mtd_nums, int dev_count, int substripe_size);
int mtd_raid_destroy(struct mtd_raid *mtd_raid);

int mtd_raid_read(struct mtd_info *mtd, loff_t from, size_t len, size_t * retlen, u_char * buf);
int mtd_raid_write(struct mtd_info *mtd, loff_t to, size_t len, size_t * retlen, const u_char * buf);
int mtd_raid_erase(struct mtd_info *mtd, struct erase_info *instr);
int mtd_raid_read_oob(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops);
int mtd_raid_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len);
int mtd_raid_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len);
void mtd_raid_resume(struct mtd_info *mtd);
int mtd_raid_suspend(struct mtd_info *mtd);
void mtd_raid_sync(struct mtd_info *mtd);
int mtd_raid_read_oob(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops);
int mtd_raid_write_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops);
int mtd_raid_block_isbad(struct mtd_info *mtd, loff_t ofs);
int mtd_raid_block_markbad(struct mtd_info *mtd, loff_t ofs);

/* raid_io.c */
int mtd_raid_dev_thread(void *u);
int mtd_raid_ctx_init(struct mtd_raid_ctx* ctx);
int mtd_raid_ctx_wait(struct mtd_raid_ctx *ctx);
void mtd_raid_ctx_destroy(struct mtd_raid_ctx *ctx);
int mtd_raid_ctx_retry(struct mtd_raid_ctx *ctx, int i_copy);

int mtd_raid_dev_read(struct mtd_raid_ctx *ctx, struct mtd_raid_dev *raid_dev,
		      loff_t from, size_t len, size_t *retlen, u_char *buf);
int mtd_raid_dev_read_oob(struct mtd_raid_ctx *ctx, struct mtd_raid_dev *raid_dev,
			  loff_t from, size_t len, struct mtd_oob_ops *ops);
int mtd_raid_dev_erase(struct mtd_raid_dev *raid_dev, struct erase_info *erase);

/* raid_single.c */
extern const struct mtd_raid_operations mtd_raid_single_ops;
struct mtd_raid *mtd_raid_single_create(int *mtd_nums, int dev_count, size_t substripe_size);

/* raid0.c */
extern const struct mtd_raid_operations mtd_raid0_ops;
struct mtd_raid *mtd_raid0_create(int *mtd_nums, int dev_count, size_t substripe_size);

/* raid1.c */
extern const struct mtd_raid_operations mtd_raid1_ops;
struct mtd_raid *mtd_raid1_create(int *mtd_nums, int dev_count, size_t substripe_size);

#endif			/* __MTD_RAID_H */
