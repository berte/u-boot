/*	$NetBSD: ufs.c,v 1.73 2015/09/01 06:12:04 dholland Exp $	*/

/*-
 * Copyright (c) 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * The Mach Operating System project at Carnegie-Mellon University.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 * Copyright (c) 1990, 1991 Carnegie Mellon University
 * All Rights Reserved.
 *
 * Author: David Golub
 *
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution (at) CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 *
 * Stand-alone file reading package for UFS and LFS filesystems.
 *
 * UFS/FFS filesystem ported to u-boot by
 * berte <behzaterte at yandex.com>
 *
 * SPDX-License-Identifier:>GPL-2.0+
 */

#include <common.h>
#include <malloc.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <linux/stat.h>
#include <ufs/ufs.h>

#include "ffs_common.h"

struct open_file *gFile = NULL;

off_t
ffs_seek(struct open_file *f, off_t offset, int where)
{
    struct file *fp = (struct file *)f->f_fsdata;
 
    switch (where) {
	case SEEK_SET:
	    fp->f_seekp = offset;
	    break;
	case SEEK_CUR:
	    fp->f_seekp += offset;
	    break;
	case SEEK_END:
	    fp->f_seekp = fp->f_di.di_size - offset;
	    break;
	default:
	    return -1;
    }
    return fp->f_seekp;
}
int ffs_close(struct open_file *f)
{
    struct file *fp = (struct file *)f->f_fsdata;

    f->f_fsdata = NULL;
    if (fp == NULL)
        return 0;

    if (fp->f_buf)
        dealloc(fp->f_buf, fp->f_fs->fs_bsize);

    dealloc(fp->f_fs, SBLOCKSIZE);
    dealloc(fp, sizeof(struct file));

    return 0;
}



void ffs_ls(struct open_file *f, const char *pattern)
{
    struct file *fp = (struct file *)f->f_fsdata;
    char *buf;
    size_t buf_size;
    lsentry_t *names = NULL;

    fp->f_seekp = 0;
    while (fp->f_seekp < (off_t)fp->f_di.di_size) {
        struct direct  *dp, *edp;
        int rc = buf_read_file(f, &buf, &buf_size);
        if (rc)
            goto out;
        /* some firmware might use block size larger than DEV_BSIZE */
        if (buf_size < UFS_DIRBLKSIZ)
            goto out;

        dp = (struct direct *)buf;
        edp = (struct direct *)(buf + buf_size);

        for (; dp < edp; dp = (void *)((char *)dp + dp->d_reclen)) {
            const char *t;
            if (dp->d_ino ==  0)
                continue;

            if (dp->d_type >= NELEM(typestr) ||
                !(t = typestr[dp->d_type])) {
                /*
                 * This does not handle "old"
                 * filesystems properly. On little
                 * endian machines, we get a bogus
                 * type name if the namlen matches a
                 * valid type identifier. We could
                 * check if we read namlen "0" and
                 * handle this case specially, if
                 * there were a pressing need...
                 */
                printf("bad dir entry\n");
                goto out;
            }
            lsadd(&names, pattern, dp->d_name, strlen(dp->d_name),
                dp->d_ino, t);
        }
        fp->f_seekp += buf_size;
    }
    lsprint(names);
out:    lsfree(names);
}

int
ffs_read(struct open_file *f, void *start, size_t size, size_t *resid)
{
    struct file *fp = (struct file *)f->f_fsdata;
    size_t csize;
    char *buf;
    size_t buf_size;
    int rc = 0;
    char *addr = start;

    while (size != 0) {
        if (fp->f_seekp >= (off_t)fp->f_di.di_size)
            break;

        rc = buf_read_file(f, &buf, &buf_size);
        if (rc)
            break;

        csize = size;
        if (csize > buf_size)
            csize = buf_size;

        memcpy(addr, buf, csize);

        fp->f_seekp += csize;
        addr += csize;
        size -= csize;
    }
    if (resid)
        *resid = size;
    return rc;
}

int
ffs_open(const char *path, struct open_file *f)
{
#ifndef LIBSA_FS_SINGLECOMPONENT
    const char *cp, *ncp;
    int c;
#endif
    ino32_t inumber;
    struct file *fp;
    FS *fs;
    int rc;
#ifndef LIBSA_NO_FS_SYMLINK
    ino32_t parent_inumber;
    int nlinks = 0;
    char namebuf[MAXPATHLEN+1];
    char *buf;
#endif

    /* allocate file system specific data structure */
    fp = alloc(sizeof(struct file));
    memset(fp, 0, sizeof(struct file));
    f->f_fsdata = (void *)fp;

    /* allocate space and read super block */
    fs = alloc(SBLOCKSIZE);
    fp->f_fs = fs;
    twiddle();

#ifdef LIBSA_FFSv2
    rc = ffs_find_superblock(f, fs);
    if (rc)
        goto out;
#else
    {
        size_t buf_size = 0;
        /*rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
            SBLOCKOFFSET / DEV_BSIZE, SBLOCKSIZE, fs, &buf_size);
        if (rc)
            goto out;*/
        if (buf_size != SBLOCKSIZE ||
#ifdef LIBSA_FFS
            fs->lfs_version != REQUIRED_LFS_VERSION ||
#endif
            fs->fs_magic != FS_MAGIC) {
            rc = EINVAL;
            goto out;
        }
    }
#if defined(LIBSA_LFS) && REQUIRED_LFS_VERSION == 2
    /*
     * XXX  We should check the second superblock and use the eldest
     *  of the two.  See comments near the top of lfs_mountfs()
     *  in sys/ufs/lfs/lfs_vfsops.c.
     *      This may need a LIBSA_LFS_SMALL check as well.
     */
#endif
#if defined(LIBSA_LFS)
    fs->lfs_is64 = 0;
    fs->lfs_dobyteswap = 0;
    fs->lfs_hasolddirfmt = (fs->fs_maxsymlinklen <= 0);
#endif
#endif

#ifdef LIBSA_FFSv1
    ffs_oldfscompat(fs);
#endif

    if (fs->fs_bsize > MAXBSIZE ||
        (size_t)fs->fs_bsize < sizeof(FS)) {
        rc = EINVAL;
        goto out;
    }

    /*
     * Calculate indirect block levels.
     */
    {
        indp_t mult;
        int ln2;

        /*
         * We note that the number of indirect blocks is always
         * a power of 2.  This lets us use shifts and masks instead
         * of divide and remainder and avoinds pulling in the
         * 64bit division routine into the boot code.
         */
        mult = FFS_NINDIR(fs);
#ifdef DEBUG
        if (mult & (mult - 1)) {
            /* Hummm was't a power of 2 */
            rc = EINVAL;
            goto out;
        }
#endif
        for (ln2 = 0; mult != 1; ln2++)
            mult >>= 1;

        fp->f_nishift = ln2;
    }

    /* alloc a block sized buffer used for all fs transfers */
    fp->f_buf = alloc(fs->fs_bsize);
    inumber = UFS_ROOTINO;
    if ((rc = read_inode(inumber, f)) != 0)
        goto out;

#ifndef LIBSA_FS_SINGLECOMPONENT
    cp = path;
    while (*cp) {

        /*
         * Remove extra separators
         */
        while (*cp == '/')
            cp++;
        if (*cp == '\0')
            break;

        /*
         * Check that current node is a directory.
         */
        if ((fp->f_di.di_mode & IFMT) != IFDIR) {
            rc = ENOTDIR;
            goto out;
        }

        /*
         * Get next component of path name.
         */
        ncp = cp;
        while ((c = *cp) != '\0' && c != '/')
            cp++;

        /*
         * Look up component in current directory.
         * Save directory inumber in case we find a
         * symbolic link.
         */
#ifndef LIBSA_NO_FS_SYMLINK
        parent_inumber = inumber;
#endif
        rc = search_directory(ncp, cp - ncp, f, &inumber);
        if (rc)
            goto out;

        /*
         * Open next component.
         */
        if ((rc = read_inode(inumber, f)) != 0)
            goto out;

#ifndef LIBSA_NO_FS_SYMLINK
        /*
         * Check for symbolic link.
         */
        if ((fp->f_di.di_mode & IFMT) == IFLNK) {
            int link_len = fp->f_di.di_size;
            int len;

            len = strlen(cp);

            if (link_len + len > MAXPATHLEN ||
                ++nlinks > MAXSYMLINKS) {
                rc = ENOENT;
                goto out;
            }

            memmove(&namebuf[link_len], cp, len + 1);

            if (link_len < fs->fs_maxsymlinklen) {
                memcpy(namebuf, fp->f_di.di_db, link_len);
            } else {
                /*
                 * Read file for symbolic link
                 */
                //size_t buf_size;
                indp_t  disk_block;

                buf = fp->f_buf;
                rc = block_map(f, (indp_t)0, &disk_block);
                if (rc)
                    goto out;

                twiddle();
                /*rc = DEV_STRATEGY(f->f_dev)(f->f_devdata,
                    F_READ, FSBTODB(fs, disk_block),
                    fs->fs_bsize, buf, &buf_size);*/
                if (rc)
                    goto out;

                memcpy(namebuf, buf, link_len);
            }

            /*
             * If relative pathname, restart at parent directory.
             * If absolute pathname, restart at root.
             */
            cp = namebuf;
            if (*cp != '/')
                inumber = parent_inumber;
            else
                inumber = (ino32_t)UFS_ROOTINO;

            if ((rc = read_inode(inumber, f)) != 0)
                goto out;
        }
#endif  /* !LIBSA_NO_FS_SYMLINK */
    }

    /*
     * Found terminal component.
     */
    rc = 0;

#else /* !LIBSA_FS_SINGLECOMPONENT */

    /* look up component in the current (root) directory */
    rc = search_directory(path, strlen(path), f, &inumber);
    if (rc)
        goto out;

    /* open it */
    rc = read_inode(inumber, f);

#endif /* !LIBSA_FS_SINGLECOMPONENT */

    fp->f_seekp = 0;        /* reset seek pointer */

out:
    if (rc)
        ffs_close(f);
#ifdef FSMOD        /* Only defined for lfs */
    else
        fsmod = FSMOD;
#endif
    return rc;
}

int
ffs_stat(struct open_file *f, struct stat *sb)
{
    struct file *fp = (struct file *)f->f_fsdata;
    
    /* only important stuff */
    memset(sb, 0, sizeof *sb);
    sb->st_mode = fp->f_di.di_mode;
    sb->st_uid = fp->f_di.di_uid;
    sb->st_gid = fp->f_di.di_gid;
    sb->st_size = fp->f_di.di_size;
    return 0;
}



/*
 * wrappers for fstype_info
 *
 */
int ufs_ls(const char *pattern)
{
   ffs_ls(gFile, pattern);
   return 0; 
}

int ufs_size(const char *filename, loff_t *size)
{
    struct stat st;

    ffs_stat(gFile, &st);

    *size = st.st_size;
    return 0;
}

int ufs_read(const char *filename, void *buf, loff_t offset, loff_t len, loff_t *len_read)
{
    int error;
    size_t sz;

    error = ffs_open(filename, gFile);
    if (!error)
    {
	if (offset != -1)
	    ffs_seek(gFile, offset, SEEK_SET);

        error = ffs_read(gFile, buf, len, &sz);
        debug("ffs_read return with %d, read byte size: %d", error, (int)sz);
    }

    return error;
}

void ufs_close(void)
{
    int ret = 0;
    ret = ffs_close(gFile);

    printf("ffs_close return value is %d\n", ret);
}

int  ufs_exists(const char *filename)
{
    int error;
 
    error = ffs_open(filename, gFile);
    
    return error;
}

