/*
 * Copyright 2015, see mtd/mtd-raid for licensing and copyright details
 */
#ifndef __MTD_RAID_USER_H__
#define __MTD_RAID_USER_H__

#include <linux/types.h>
#include <linux/magic.h>

/* ioctl's command */
#define MTD_RAID_IOC_CREATE		_IOW(0xFE, 1, struct mtd_raid_create_req)
#define MTD_RAID_IOC_DESTROY		_IOW(0xFE, 2, struct mtd_raid_destroy_req)

enum mtd_raid_level {
	MTD_RAID_LEVEL_SINGLE = 0,
	MTD_RAID_LEVEL_RAID0,
	MTD_RAID_LEVEL_RAID1,
	MTD_RAID_LEVEL_MAX
};

struct mtd_raid_create_req {
	__u8 raid_level;
	__u8 reserved[3];
	__u32 dev_count;
	__u64 substripe_size;
	__u32 mtd_nums[0];
};

struct mtd_raid_destroy_req {
	__u32 mtd_num;
};

#endif				/* __MTD_RAID_USER_H__ */
