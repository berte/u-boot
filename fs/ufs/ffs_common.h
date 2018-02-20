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

#ifndef __FFS_COMMON_H__
#define __FFS_COMMON_H__

#include "fs.h"

#define UFS_NXADDR 2
#define UFS_NDADDR 12      /* Direct addresses in inode. */
#define UFS_NIADDR 3       /* Indirect addresses in inode. */

#define indp_t      int32_t
#define indp_t      int64_t

#define FS_MAGIC        	0x070162/*LFS_MAGIC*/
#define SBLOCKSIZE      	8192/*LFS_SBPAD*/
#define SBLOCKOFFSET    	8192/*LFS_LABELPAD*/
#define FS_UFS1_MAGIC   	0x011954    /* UFS1 fast file system magic number */
#define FS_UFS2_MAGIC   	0x19540119  /* UFS2 fast file system magic number */
#define FS_UFS1_MAGIC_SWAPPED   0x54190100
#define FS_UFS2_MAGIC_SWAPPED   0x19015419

#define SBLOCK_FLOPPY      0
#define SBLOCK_UFS1     8192
#define SBLOCK_UFS2    	65536
#define SBLOCK_PIGGY  	262144
#define SBLOCKSIZE      8192
#define DEV_BSIZE   	512

#ifndef SEEK_SET
#define	SEEK_SET	0	/* set file offset to offset */
#endif
#ifndef SEEK_CUR
#define	SEEK_CUR	1	/* set file offset to current plus offset */
#endif
#ifndef SEEK_END
#define	SEEK_END	2	/* set file offset to EOF plus offset */
#endif

/*
 * NB: Do not, under any circumstances, look for an ffsv1 filesystem at
 * SBLOCK_UFS2.  Doing so will find the wrong superblock for filesystems
 * with a 64k block size.
 */
#define SBLOCKSEARCH \
    { SBLOCK_UFS2, SBLOCK_UFS1, SBLOCK_FLOPPY, SBLOCK_PIGGY, -1 }


#define FS_OKAY     		0x7c269d38  /* superblock checksum */
#define FS_42INODEFMT   	-1      /* 4.2BSD inode format */
#define FS_44INODEFMT   	2       /* 4.4BSD inode format */
#define LN2_IND_CACHE_SZ    	6
#define IND_CACHE_SZ        	(1 << LN2_IND_CACHE_SZ)
#define IND_CACHE_MASK      	(IND_CACHE_SZ - 1)

#define UFS_DIRBLKSIZ       	512 
#define FFS_MAXNAMLEN       	255
#define APPLEUFS_DIRBLKSIZ  	1024
#define MAXPHYS     		65536       /* max I/O transfer size */
#define MAXPATHLEN  		1024/*PATH_MAX*/
#define MAXSYMLINKS 		32
#define MAXBSIZE    		MAXPHYS

#define ffs_blkoff(fs, loc) /* calculates (loc % fs->fs_bsize) */ \
        ((loc) & (fs)->fs_qbmask)

#define ffs_lblkno(fs, loc) /* calculates (loc / fs->fs_bsize) */ \
        ((loc) >> (fs)->fs_bshift)

#define ffs_blksize(fs, ip, lbn) \
    (((lbn) >= UFS_NDADDR || (ip)->i_size >= ffs_lblktosize(fs, (lbn) + 1)) \
        ? (fs)->fs_bsize \
        : ((int32_t)ffs_fragroundup(fs, ffs_blkoff(fs, (ip)->i_size))))

#define ffs_sblksize(fs, size, lbn) \
    (((lbn) >= UFS_NDADDR || (size) >= ((lbn) + 1) << (fs)->fs_bshift) \
      ? (fs)->fs_bsize \
      : ((int32_t)ffs_fragroundup(fs, ffs_blkoff(fs, (uint64_t)(size)))))


#define DEV_NAME(d)     ((d)->dv_name)
#define FFS_FSBTODB(fs, b)  ((b) << (fs)->fs_fsbtodb)
#define FFS_DBTOFSB(fs, b)  ((b) >> (fs)->fs_fsbtodb)

#ifndef FSBTODB
    #define FSBTODB(fs, indp) FFS_FSBTODB(fs, indp)
#endif
#ifndef UFS_NINDIR
    #define UFS_NINDIR FFS_NINDIR
#endif
#ifndef ufs_blkoff
    #define ufs_blkoff ffs_blkoff
#endif
#ifndef ufs_lblkno
    #define ufs_lblkno ffs_lblkno
#endif

#define FFS_NINDIR(fs)  ((fs)->fs_nindir)
#define    cgbase(fs, c)   (((daddr_t)(fs)->fs_fpg) * (c))

#define cgstart_ufs1(fs, c) \
    (cgbase(fs, c) + (fs)->fs_old_cgoffset * ((c) & ~((fs)->fs_old_cgmask)))

#define cgstart_ufs2(fs, c) cgbase((fs), (c))

#define cgstart(fs, c) ((fs)->fs_magic == FS_UFS2_MAGIC \
                ? cgstart_ufs2((fs), (c)) : cgstart_ufs1((fs), (c)))

#define cgdmin(fs, c)   (cgstart(fs, c) + (fs)->fs_dblkno)  /* 1st data */
#define cgimin(fs, c)   (cgstart(fs, c) + (fs)->fs_iblkno)  /* inode blk */
#define cgsblock(fs, c) (cgstart(fs, c) + (fs)->fs_sblkno)  /* super blk */
#define cgtod(fs, c)    (cgstart(fs, c) + (fs)->fs_cblkno)  /* cg block */

#define ino_to_cg(fs, x)    ((x) / (fs)->fs_ipg)

#define ino_to_fsba(fs, x)                      \
    ((daddr_t)(cgimin(fs, ino_to_cg(fs, x)) +           \
        (ffs_blkstofrags((fs), (((x) % (fs)->fs_ipg) / FFS_INOPB(fs))))))

#define ino_to_fsbo(fs, x)  ((x) % FFS_INOPB(fs))

/* f_flags values */
#define F_READ      0x0001  /* file opened for reading */
#define F_WRITE     0x0002  /* file opened for writing */
#define F_RAW       0x0004  /* raw device open - no file system */
#define F_NODEV     0x0008  /* network open - no device */

#define UFS_ROOTINO ((ino_t)2)
#define UFS_ROOTINO ((ino_t)2)

/* File permissions. */
#define IEXEC       0000100     /* Executable. */
#define IWRITE      0000200     /* Writable. */
#define IREAD       0000400     /* Readable. */
#define ISVTX       0001000     /* Sticky bit. */
#define ISGID       0002000     /* Set-gid. */
#define ISUID       0004000     /* Set-uid. */

/* File types. */
#define IFMT        0170000     /* Mask of file type. */
#define IFIFO       0010000     /* Named pipe (fifo). */
#define IFCHR       0020000     /* Character device. */
#define IFDIR       0040000     /* Directory file. */
#define IFBLK       0060000     /* Block device. */
#define IFREG       0100000     /* Regular file. */
#define IFLNK       0120000     /* Symbolic link. */
#define IFSOCK      0140000     /* UNIX domain socket. */
#define IFWHT       0160000     /* Whiteout. */

#define TWIDDLE_CHARS   "|/-\\"

struct ufs1_dinode {
    uint16_t    di_mode;    /*   0: IFMT, permissions; see below. */
    int16_t     di_nlink;   /*   2: File link count. */
    uint16_t    di_oldids[2];   /*   4: Ffs: old user and group ids. */
    uint64_t    di_size;    /*   8: File byte count. */
    int32_t     di_atime;   /*  16: Last access time. */
    int32_t     di_atimensec;   /*  20: Last access time. */
    int32_t     di_mtime;   /*  24: Last modified time. */
    int32_t     di_mtimensec;   /*  28: Last modified time. */
    int32_t     di_ctime;   /*  32: Last inode change time. */
    int32_t     di_ctimensec;   /*  36: Last inode change time. */
    int32_t     di_db[UFS_NDADDR]; /*  40: Direct disk blocks. */
    int32_t     di_ib[UFS_NIADDR]; /*  88: Indirect disk blocks. */
    uint32_t    di_flags;   /* 100: Status flags (chflags). */
    uint32_t    di_blocks;  /* 104: Blocks actually held. */
    int32_t     di_gen;     /* 108: Generation number. */
    uint32_t    di_uid;     /* 112: File owner. */
    uint32_t    di_gid;     /* 116: File group. */
    uint64_t    di_modrev;  /* 120: i_modrev for NFSv4 */
};

struct ufs2_dinode {
    uint16_t    di_mode;    /*   0: IFMT, permissions; see below. */
    int16_t     di_nlink;   /*   2: File link count. */
    uint32_t    di_uid;     /*   4: File owner. */
    uint32_t    di_gid;     /*   8: File group. */
    uint32_t    di_blksize; /*  12: Inode blocksize. */
    uint64_t    di_size;    /*  16: File byte count. */
    uint64_t    di_blocks;  /*  24: Bytes actually held. */
    int64_t     di_atime;   /*  32: Last access time. */
    int64_t     di_mtime;   /*  40: Last modified time. */
    int64_t     di_ctime;   /*  48: Last inode change time. */
    int64_t     di_birthtime;   /*  56: Inode creation time. */
    int32_t     di_mtimensec;   /*  64: Last modified time. */
    int32_t     di_atimensec;   /*  68: Last access time. */
    int32_t     di_ctimensec;   /*  72: Last inode change time. */
    int32_t     di_birthnsec;   /*  76: Inode creation time. */
    int32_t     di_gen;     /*  80: Generation number. */
    uint32_t    di_kernflags;   /*  84: Kernel flags. */
    uint32_t    di_flags;   /*  88: Status flags (chflags). */
    int32_t     di_extsize; /*  92: External attributes block. */
    int64_t     di_extb[UFS_NXADDR];/* 96: External attributes block. */
    int64_t     di_db[UFS_NDADDR]; /* 112: Direct disk blocks. */
    int64_t     di_ib[UFS_NIADDR]; /* 208: Indirect disk blocks. */
    uint64_t    di_modrev;  /* 232: i_modrev for NFSv4 */
    int64_t     di_spare[2];    /* 240: Reserved; currently unused */
};

struct _ufs_dinode
{
    struct ufs1_dinode v1;
    struct ufs2_dinode v2;
};

extern int ndevs;       /* number of elements in devsw[] */
typedef uint32_t ino32_t;
typedef struct fs FS;
typedef struct _ufs_dinode ufs_dinode;

struct file {
     off_t    f_seekp;    		/* seek pointer */
     FS      *f_fs;      		/* pointer to super-block */
     ufs_dinode f_di;       	/* copy of on-disk inode */
     uint    f_nishift;  		/* for blocks in indirect block */
     indp_t  f_ind_cache_block;
     indp_t  f_ind_cache[IND_CACHE_SZ];

     char    *f_buf;     		/* buffer for data block */
     size_t  f_buf_size; 		/* size of data block */
     daddr_t f_buf_blkno;    		/* block number of data block */
};

struct open_file {
    int     f_flags;    /* see F_* below */
#if !defined(LIBSA_SINGLE_DEVICE)
    const struct devsw  *f_dev; /* pointer to device operations */
#endif
    void        *f_devdata; /* device specific data */
#if !defined(LIBSA_SINGLE_FILESYSTEM)
    const struct fs_ops *f_ops; /* pointer to file system operations */
#endif
    void        *f_fsdata;  /* file system specific data */
#if !defined(LIBSA_NO_RAW_ACCESS)
    off_t       f_offset;   /* current file offset (F_RAW) */
#endif
};

struct lsentry {
    struct lsentry *e_next;
    uint32_t e_ino;
    const char *e_type;
    char    e_name[1];
};

typedef struct lsentry lsentry_t;

#define d_ino d_fileno
struct  direct {
    u_int32_t d_fileno;     /* inode number of entry */
    u_int16_t d_reclen;     /* length of this record */
    u_int8_t  d_type;       /* file type, see below */
    u_int8_t  d_namlen;     /* length of string in d_name */
    char      d_name[FFS_MAXNAMLEN + 1];/* name with length <= FFS_MAXNAMLEN */
};

#define NELEM(x) (sizeof (x) / sizeof(*x))

static const char    *const typestr[] = {
    "unknown",
    "FIFO",
    "CHR",
    0,
    "DIR",
    0,
    "BLK",
    0,
    "REG",
    0,
    "LNK",
    0,
    "SOCK",
    0,
    "WHT"
};

void dealloc(void *, size_t );
void *alloc(size_t);
void twiddle(void);
int fnmatch(const char *, const char *);
void lsadd(lsentry_t **, const char *, const char *, size_t, uint32_t, const char *);
void lsprint(lsentry_t *); 
void lsfree(lsentry_t *);
int read_inode(ino32_t, struct open_file *);
int block_map(struct open_file *, indp_t, indp_t *);
int buf_read_file(struct open_file *, char **, size_t *);
int search_directory(const char *, int, struct open_file *, ino32_t *);
void ffs_oldfscompat(FS *);
int ffs_find_superblock(struct open_file *, FS *);

int64_t get_ufs_version(struct open_file*);
#endif /* __FFS_COMMON_H__  */
