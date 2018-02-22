/*
 *
 * UFS/FFSv1/FFSv2 filesystem support
 * Created by berte <behzaterte at yandex.com>
 * SPDX-License-Identifier:   BSD 2-Clause License
 */

#ifndef __UFS_H__
#define __UFS_H__

int  ufs_probe(struct blk_desc *, disk_partition_t *);
int  ufs_ls(const char *);
int  ufs_exists(const char *);
int  ufs_size(const char *, loff_t *);
int  ufs_read(const char *, void *, loff_t, loff_t, loff_t *);
void ufs_close(void);

#endif /* __UFS_H__  */

