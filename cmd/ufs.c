/*-
 * Command for UFS filesystem support.
 * Created by berte <behzaterte at yandex.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause 
 */

#include <fs.h>

static int do_ffsls(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    return do_ls(cmdtp, flag, argc, argv, FS_TYPE_UFS);
}

int do_ffsload(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return do_load(cmdtp, flag, argc, argv, FS_TYPE_UFS);
}

int do_ffssize(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return do_size(cmdtp, flag, argc, argv, FS_TYPE_UFS);
}

U_BOOT_CMD(
	ffssize,       4,      0,      do_ffssize,
	"determine a file's size",
    "<interface> <dev[:part]> <filename>\n"
    "    - Find file 'filename' from 'dev' on 'interface'\n"
	"      and determine its size."
);

U_BOOT_CMD(
	ffsls,	4,	1,	do_ffsls,
	"list files in a directory (default /)",
	"<interface> <type> <dev[:part]> [directory]\n"
	"    - list files from 'dev' on 'interface' in a 'directory'"
)

U_BOOT_CMD(
	ffsload,	6,	0,	do_ffsload,
	"load binary file from a ffs filesystem",
	"<interface> <type> [<dev[:part]> [addr [filename [bytes [pos]]]]]\n"
	"    - load binary file 'filename' from 'dev' on 'interface'\n"
	"      to address 'addr' from ffs filesystem."
)
