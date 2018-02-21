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
 * SPDX-License-Identifier:     BSD 2-Clause License
 */


#include <common.h>
#include "ffs_common.h"

char  twiddle_toggle;
struct fl {
    unsigned int    size;
    struct fl   *next;
} *freelist;

void
twiddle(void)
{
    static int pos;

    if (!twiddle_toggle) {
        /*putchar(TWIDDLE_CHARS[pos++ & 3]);
        putchar('\b');*/
        putc(TWIDDLE_CHARS[pos++ & 3]);
        putc('\b');
    }

}

void 
dealloc(void *ptr, size_t size)
{
    struct fl *f = (struct fl *)(void *) ((char *)(void *)ptr - sizeof(unsigned int));
#ifdef DEBUG
    if (size > (size_t)f->size) {
        printf("dealloc %zu bytes @%lx, should be <=%u\n",
            size, (u_long)ptr, f->size);
    }

    if (ptr < (void *)HEAP_START)
        printf("dealloc: %lx before start of heap.\n", (u_long)ptr);

#ifdef HEAP_LIMIT
    if (ptr > (void *)HEAP_LIMIT)
        printf("dealloc: %lx beyond end of heap.\n", (u_long)ptr);
#endif
#endif /* DEBUG */
    /* put into freelist */
    f->next = freelist;
    freelist = f;
}

static char *top = NULL;

void *
alloc(size_t size)
{
    struct fl **f = &freelist, **bestf = NULL;
    unsigned int bestsize = 0xffffffff; /* greater than any real size */
    char *help = NULL;
    int failed;

    /* scan freelist */
    while (*f) {
        if ((size_t)(*f)->size >= size) {
            if ((size_t)(*f)->size == size) /* exact match */
                goto found;

            if ((*f)->size < bestsize) {
                /* keep best fit */
                bestf = f;
                bestsize = (*f)->size;
            }
        }
        f = &((*f)->next);
    }

    /* no match in freelist if bestsize unchanged */
    failed = (bestsize == 0xffffffff);

    if (failed) { /* nothing found */
        /*
         * allocate from heap, keep chunk len in
         * first word
         */
        //help = top;

        /* make _sure_ the region can hold a struct fl. */
        if (size < sizeof (struct fl *))
            size = sizeof (struct fl *);
        top += sizeof(unsigned int) + size;
#ifdef HEAP_LIMIT
        if (top > (char *)HEAP_LIMIT)
            panic("heap full (%p+%zu)", help, size);
#endif
        *(unsigned int *)(void *)help = (unsigned int)(size);
        return help + sizeof(unsigned int);
    }

    /* we take the best fit */
    f = bestf;

found:
    /* remove from freelist */
    help = (char *)(void *)*f;
    *f = (*f)->next;
    return help + sizeof(unsigned int);
}

int 
fnmatch(const char *fname, const char *pattern)
{
    char fc, pc;

    do {
        fc = *fname++;
        pc = *pattern++;
        if (!fc && !pc)
            return 1;
        if (pc == '?' && fc)
            pc = fc;
    } while (fc == pc);

    if (pc != '*')
        return 0;
    /*
     * Too hard (and unnecessary really) too check for "*?name" etc....
     * "**" will look for a '*' and "*?" a '?'
     */
    pc = *pattern++;
    if (!pc)
        return 1;
    while ((fname = strchr(fname, pc)))
        if (fnmatch(++fname, pattern))
            return 1;
    return 0;
}
void
lsadd(lsentry_t **names, const char *pattern, const char *name, size_t namelen,
    uint32_t ino, const char *type)
{
    lsentry_t *n, **np;

    if (pattern && !fnmatch(name, pattern))
        return;

    n = alloc(sizeof *n + namelen);
    if (!n) {
        printf("%d: %.*s (%s)\n", ino, (int)namelen, name, type);
        return;
    }

    n->e_ino = ino;
    n->e_type = type;
    memcpy(n->e_name, name, namelen);
    n->e_name[namelen] = '\0';

    for (np = names; *np; np = &(*np)->e_next) {
        if (strcmp(n->e_name, (*np)->e_name) < 0)
            break;
    }
    n->e_next = *np;
    *np = n;
}

void
lsprint(lsentry_t *names) {
    if (!names) {
        printf("not found\n");
        return;
    }
    do {
        lsentry_t *n = names;
        printf("%d: %s (%s)\n", n->e_ino, n->e_name, n->e_type);
        names = n->e_next;
    } while (names);
}

void
lsfree(lsentry_t *names) {
    if (!names)
        return;
    do {
        lsentry_t *n = names;
        names = n->e_next;
        dealloc(n, 0);
    } while (names);
}

int
read_inode(ino32_t inumber, struct open_file *f)
{
    int64_t version = get_ufs_version(f);
    struct file *fp = (struct file *)f->f_fsdata;
    FS *fs = fp->f_fs;
    char *buf;
    size_t rsize = 0;
    int rc;
    daddr_t inode_sector = 0; /* XXX: gcc */
#ifdef LIBSA_LFS
    struct ufs_dinode *dip;
    int cnt;
#endif

#ifdef LIBSA_LFS
    if (inumber == LFS_IFILE_INUM)
        inode_sector = FSBTODB(fs, lfs_sb_getidaddr(fs));
    else if ((rc = find_inode_sector(inumber, f, &inode_sector)) != 0)
        return rc;
#else
    inode_sector = FSBTODB(fs, ino_to_fsba(fs, inumber));
#endif

    /*
     * Read inode and save it.
     */
    buf = fp->f_buf;
    twiddle();
    /*rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
        inode_sector, fs->fs_bsize, buf, &rsize);
    if (rc)
        return rc;*/
    if (rsize != fs->fs_bsize)
        return EIO;

#ifdef LIBSA_LFS
    cnt = INOPBx(fs);
    dip = (struct ufs_dinode *)buf + (cnt - 1);
    for (; dip->di_inumber != inumber; --dip) {
        /* kernel code panics, but boot blocks which panic are Bad. */
        if (--cnt == 0)
            return EINVAL;
    }
    fp->f_di = *dip;
#else
    if (version == FS_UFS1_MAGIC) 
    	fp->f_di.v1 = ((struct ufs1_dinode *)buf)[ino_to_fsbo(fs, inumber)];
    fp->f_di.v2 = ((struct ufs2_dinode *)buf)[ino_to_fsbo(fs, inumber)];
#endif

    /*
     * Clear out the old buffers
     */
    fp->f_ind_cache_block = ~0;
    fp->f_buf_blkno = -1;
    return rc;
}

/*
 * Given an offset in a file, find the disk block number that
 * contains that block.
 */
int
block_map(struct open_file *f, indp_t file_block, indp_t *disk_block_p)
{
    int64_t version = get_ufs_version(f);
    struct file *fp = (struct file *)f->f_fsdata;
    FS *fs = fp->f_fs;
    uint level;
    indp_t ind_cache;
    indp_t ind_block_num;
    size_t rsize = 0;
    indp_t *buf = (void *)fp->f_buf;

    /*
     * Index structure of an inode:
     *
     * di_db[0..UFS_NDADDR-1]   hold block numbers for blocks
     *          0..UFS_NDADDR-1
     *
     * di_ib[0]     index block 0 is the single indirect block
     *          holds block numbers for blocks
     *          UFS_NDADDR .. UFS_NDADDR + UFS_NINDIR(fs)-1
     *
     * di_ib[1]     index block 1 is the double indirect block
     *          holds block numbers for INDEX blocks for blocks
     *          UFS_NDADDR + UFS_NINDIR(fs) ..
     *          UFS_NDADDR + UFS_NINDIR(fs) + UFS_NINDIR(fs)**2 - 1
     *
     * di_ib[2]     index block 2 is the triple indirect block
     *          holds block numbers for double-indirect
     *          blocks for blocks
     *          UFS_NDADDR + UFS_NINDIR(fs) + UFS_NINDIR(fs)**2 ..
     *          UFS_NDADDR + UFS_NINDIR(fs) + UFS_NINDIR(fs)**2
     *              + UFS_NINDIR(fs)**3 - 1
     */

    if (file_block < UFS_NDADDR) {
        /* Direct block. */
        *disk_block_p = (version == FS_UFS1_MAGIC) ? fp->f_di.v1.di_db[file_block] : fp->f_di.v2.di_db[file_block];
        return 0;
    }

    file_block -= UFS_NDADDR;

    ind_cache = file_block >> LN2_IND_CACHE_SZ;
    if (ind_cache == fp->f_ind_cache_block) {
        *disk_block_p = fp->f_ind_cache[file_block & IND_CACHE_MASK];
        return 0;
    }

    for (level = 0;;) {
        level += fp->f_nishift;
        if (file_block < (indp_t)1 << level)
            break;
        if (level > UFS_NIADDR * fp->f_nishift)
            /* Block number too high */
            return EFBIG;
        file_block -= (indp_t)1 << level;
    }

    ind_block_num = (version == FS_UFS1_MAGIC) ? fp->f_di.v1.di_ib[level / fp->f_nishift - 1] :  fp->f_di.v2.di_ib[level / fp->f_nishift - 1];

    for (;;) {
        level -= fp->f_nishift;
        if (ind_block_num == 0) {
            *disk_block_p = 0;  /* missing */
            return 0;
        }

        twiddle();
        /*
         * If we were feeling brave, we could work out the number
         * of the disk sector and read a single disk sector instead
         * of a filesystem block.
         * However we don't do this very often anyway...
         */
        /*rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
            FSBTODB(fp->f_fs, ind_block_num), fs->fs_bsize,
            buf, &rsize);
        if (rc)
            return rc;*/
        if (rsize != fs->fs_bsize)
            return EIO;
        ind_block_num = buf[file_block >> level];
        if (level == 0)
            break;
        file_block &= (1 << level) - 1;
    }

    /* Save the part of the block that contains this sector */
    memcpy(fp->f_ind_cache, &buf[file_block & ~IND_CACHE_MASK],
        IND_CACHE_SZ * sizeof fp->f_ind_cache[0]);
    fp->f_ind_cache_block = ind_cache;

    *disk_block_p = ind_block_num;

    return 0;
}

/*
 * Read a portion of a file into an internal buffer.
 * Return the location in the buffer and the amount in the buffer.
 */
int
buf_read_file(struct open_file *f, char **buf_p, size_t *size_p)
{
    int64_t version = get_ufs_version(f);
    struct file *fp = (struct file *)f->f_fsdata;
    FS *fs = fp->f_fs;
    long off;
    indp_t file_block;
    size_t block_size;
    int rc;

    off = ufs_blkoff(fs, fp->f_seekp);
    file_block = ufs_lblkno(fs, fp->f_seekp);
#ifdef LIBSA_LFS
    block_size = (version == FS_UFS1_MAGIC) ? dblksize(fs, &fp->f_di.v1, file_block) :  
					    dblksize(fs, &fp->f_di.v2, file_block);
#else
    block_size = (version == FS_UFS1_MAGIC) ? ffs_sblksize(fs, (int64_t)fp->f_di.v1.di_size, file_block) :
						ffs_sblksize(fs, (int64_t)fp->f_di.v2.di_size, file_block);
#endif

    if (file_block != fp->f_buf_blkno) {
        indp_t disk_block = 0; /* XXX: gcc */
        rc = block_map(f, file_block, &disk_block);
        if (rc)
            return rc;

        if (disk_block == 0) {
            memset(fp->f_buf, 0, block_size);
            fp->f_buf_size = block_size;
        } else {
            twiddle();
            /*rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
                FSBTODB(fs, disk_block),
                block_size, fp->f_buf, &fp->f_buf_size);
            if (rc)
                return rc;*/
        }

        fp->f_buf_blkno = file_block;
    }

    /*
     * Return address of byte in buffer corresponding to
     * offset, and size of remainder of buffer after that
     * byte.
     */
    *buf_p = fp->f_buf + off;
    *size_p = block_size - off;

    /*
     * But truncate buffer at end of file.
     */
   int32_t sum = (version == FS_UFS1_MAGIC) ? (fp->f_di.v1.di_size - fp->f_seekp) : (fp->f_di.v2.di_size - fp->f_seekp); 
    if (*size_p > sum)
        *size_p = (version == FS_UFS1_MAGIC) ? (fp->f_di.v1.di_size - fp->f_seekp) : (fp->f_di.v2.di_size - fp->f_seekp);

    return 0;
}

/*
 * Search a directory for a name and return its
 * inode number.
 */
int
search_directory(const char *name, int length, struct open_file *f,
    ino32_t *inumber_p)
{
    int64_t version = get_ufs_version(f);
    struct file *fp = (struct file *)f->f_fsdata;
    struct direct *dp;
    struct direct *edp;
    char *buf;
    size_t buf_size;
    int namlen;
    int rc;

    fp->f_seekp = 0;
    int16_t di_size = (version == FS_UFS1_MAGIC) ? (off_t)fp->f_di.v1.di_size : (off_t)fp->f_di.v2.di_size;
    while (fp->f_seekp < di_size) {
        rc = buf_read_file(f, &buf, &buf_size);
        if (rc)
            return rc;

        dp = (struct direct *)buf;
        edp = (struct direct *)(buf + buf_size);
        for (;dp < edp; dp = (void *)((char *)dp + dp->d_reclen)) {
            if (dp->d_reclen <= 0)
                break;
            if (dp->d_ino == (ino32_t)0)
                continue;
#if BYTE_ORDER == LITTLE_ENDIAN
            if (fp->f_fs->fs_maxsymlinklen <= 0)
                namlen = dp->d_type;
            else
#endif
                namlen = dp->d_namlen;
            if (namlen == length &&
                !memcmp(name, dp->d_name, length)) {
                /* found entry */
                *inumber_p = dp->d_ino;
                return 0;
            }
        }
        fp->f_seekp += buf_size;
    }
    return ENOENT;
}


/*
 * Sanity checks for old file systems.
 *
 * XXX - goes away some day.
 * Stripped of stuff libsa doesn't need.....
 */
void
ffs_oldfscompat(FS *fs)
{

    /*
     * Newer Solaris versions have a slightly incompatible
     * superblock - so always calculate this values on the fly, which
     * is good enough for libsa purposes
     */
    if (fs->fs_magic == FS_UFS1_MAGIC
#ifndef COMPAT_SOLARIS_UFS
        && fs->fs_old_inodefmt < FS_44INODEFMT
#endif
        ) {
        fs->fs_qbmask = ~fs->fs_bmask;
        fs->fs_qfmask = ~fs->fs_fmask;
    }
}

daddr_t sblock_try[] = SBLOCKSEARCH;
int
ffs_find_superblock(struct open_file *f, FS *fs)
{
    int i;

    for (i = 0; sblock_try[i] != -1; i++) {
        /*rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
            sblock_try[i] / DEV_BSIZE, SBLOCKSIZE, fs, &buf_size);
        if (rc != 0 || buf_size != SBLOCKSIZE)
            return rc;*/
        if (fs->fs_sblockloc != sblock_try[i])
            /* an alternate superblock - try again */
            continue;
        if (fs->fs_magic == FS_UFS2_MAGIC) {
            return 0;
        }
    }
    return EINVAL;
}

int64_t get_ufs_version(struct open_file *f)
{
    struct file *fp = (struct file *)f->f_fsdata;
    
    FS *fs = fp->f_fs;
    
    if (fs->fs_magic == FS_UFS1_MAGIC)
	return FS_UFS1_MAGIC;
    
    return FS_UFS2_MAGIC;
}
