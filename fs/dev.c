/*
 * This file contains some functions shared by block_dev and char_dev
 */
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/mount.h>

#include "internal.h"

/**
 * __lookup_dev  - lookup a block_device or cdev by name
 * @pathname:	special file representing the device
 * @cdevp:	cdev would be returned by cdevp
 * @bdevp:	block_device would be returned by bdevp
 *
 * Get a reference to the block_deivce or cdev at @pathname in
 * the current namespace if possible and return it.
 */
int __lookup_dev(const char *pathname, struct cdev **cdevp,
	       struct block_device **bdevp)
{
	struct inode *inode;
	struct path path;
	int error = 0;

	if (!pathname || !*pathname)
		return -EINVAL;

	error = kern_path(pathname, LOOKUP_FOLLOW, &path);
	if (error)
		return error;

	inode = d_backing_inode(path.dentry);

	/**
	 * We need at least one of bdevp and cdevp to be NULL,
	 * but cdevp and bdevp can not be both NULL.
	 */
	error = -EINVAL;
	if (!(cdevp || bdevp) || (cdevp && bdevp))
		goto out;

	if (cdevp) {
		if (!S_ISCHR(inode->i_mode)) {
			error = -EINVAL;
			goto out;
		}
	} else {
		if (!S_ISBLK(inode->i_mode)) {
			error = -ENOTBLK;
			goto out;
		}
	}
	error = -EACCES;
	if (path.mnt->mnt_flags & MNT_NODEV)
		goto out;
	error = -ENXIO;
	if (S_ISCHR(inode->i_mode)) {
		struct cdev *cdev;

		cdev = cd_acquire(inode);
		if (!cdev)
			goto out;
		*cdevp = cdev;
	} else {
		struct block_device *bdev;

		bdev = bd_acquire(inode);
		if (!bdev)
			goto out;
		*bdevp = bdev;
	}
	error = 0;
out:
	path_put(&path);
	return error;
}
