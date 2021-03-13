#include <fcntl.h>
#include <getopt.h>
#include <mtd/mtd-user.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <stdbool.h>
#include <unistd.h>

#define PROGRAM_NAME "ampere_flashcp"
#define VERSION "v1.0"

/* for debugging purposes only */
#ifdef DEBUG
#undef DEBUG
#define DEBUG(fmt, args...)                  \
  {                                          \
    log_printf(LOG_ERROR, "%d: ", __LINE__); \
    log_printf(LOG_ERROR, fmt, ##args);      \
  }
#else
#undef DEBUG
#define DEBUG(fmt, args...)
#endif

#define KB(x) ((x) / 1024)
#define PERCENTAGE(x, total) (((x)*100) / (total))

/* size of read/write buffer */
#define BUFSIZE (10 * 1024)

/* cmd-line flags */
#define FLAG_NONE      0x00
#define FLAG_VERBOSE   0x01
#define FLAG_HELP      0x02
#define FLAG_FILENAME  0x04
#define FLAG_DEVICE    0x08
#define FLAG_ERASE_ALL 0x10

/* error levels */
#define LOG_NORMAL     1
#define LOG_ERROR      2

#define common_print_version() \
do { \
	printf("%s version %s\n", PROGRAM_NAME, VERSION); \
} while (0)

static void log_printf(int level, const char *fmt, ...)
{
	FILE *fp = level == LOG_NORMAL ? stdout : stderr;
	va_list ap;
	va_start(ap, fmt);
	vfprintf(fp, fmt, ap);
	va_end(ap);
	fflush(fp);
}

static void showusage(bool error)
{
	int level = error ? LOG_ERROR : LOG_NORMAL;

	log_printf(
			level,
			"usage: %1$s [ -v | --verbose | -A | --erase-all ] <filename> <device> <offset>\n"
			"       %1$s -h | --help\n"
			"       %1$s -V | --version\n"
			"\n"
			"   -h | --help      Show this help message\n"
			"   -v | --verbose   Show progress reports\n"
			"   -A | --erase-all Erases the whole device regardless of the image "
			"size\n"
			"   -V | --version   Show version information and exit\n"
			"   <filename>       File which you want to copy to flash\n"
			"   <device>         Flash device to write to (e.g. /dev/mtd0, "
			"/dev/mtd1, etc.)\n"
			"   <offset>         The start offset. Optional, default: 0\n"
			"\n",
			PROGRAM_NAME);

	exit(error ? EXIT_FAILURE : EXIT_SUCCESS);
}

static int safe_open(const char *pathname, int flags)
{
	int fd;

	fd = open(pathname, flags);
	if (fd < 0) {
		log_printf(LOG_ERROR, "While trying to open %s", pathname);
		if (flags & O_RDWR)
			log_printf(LOG_ERROR, " for read/write access");
		else if (flags & O_RDONLY)
			log_printf(LOG_ERROR, " for read access");
		else if (flags & O_WRONLY)
			log_printf(LOG_ERROR, " for write access");
		log_printf(LOG_ERROR, ": %m\n");
		exit(EXIT_FAILURE);
	}

	return (fd);
}

static void safe_read(int fd, const char *filename, void *buf, size_t count,
		bool verbose)
{
	ssize_t result;

	result = read(fd, buf, count);
	if (count != result) {
		if (verbose)
			log_printf(LOG_NORMAL, "\n");
		if (result < 0) {
			log_printf(LOG_ERROR, "While reading data from %s: %m\n", filename);
			exit(EXIT_FAILURE);
		}
		log_printf(LOG_ERROR,
				"Short read count returned while reading from %s\n", filename);
		exit(EXIT_FAILURE);
	}
}

static void safe_rewind(int fd, const char *filename)
{
	if (lseek(fd, 0L, SEEK_SET) < 0) {
		log_printf(LOG_ERROR, "While seeking to start of %s: %m\n", filename);
		exit(EXIT_FAILURE);
	}
}

static int safe_rewind_offset(int fd, const char *filename, off_t offset)
{
	if (lseek(fd, offset, SEEK_SET) < 0) {
		log_printf(LOG_ERROR, "While seeking to start of %s: %m\n", filename);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

/******************************************************************************/

static int dev_fd = -1;
static int fil_fd = -1;
static int flags = FLAG_NONE;
struct mtd_info_user mtd;
struct erase_info_user erase;
struct stat filestat;

static void cleanup(void)
{
	if (dev_fd > 0)
		close(dev_fd);
	if (fil_fd > 0)
		close(fil_fd);
}

static int flash_erase(int dev_fd, off_t offset, const char *device)
{
	int i = 0;
	erase.start = 0;

	if (flags & FLAG_ERASE_ALL) {
		erase.length = mtd.size;
	} else {
		/* Erase from the offset */
		erase.start = offset;
		erase.length = (filestat.st_size + mtd.erasesize - 1) / mtd.erasesize;
		erase.length *= mtd.erasesize;
	}

	if (flags & FLAG_VERBOSE) {
		/* if the user wants verbose output, erase 1 block at a time and show
		 * him/her what's going on */
		int blocks = erase.length / mtd.erasesize;
		erase.length = mtd.erasesize;
		log_printf(LOG_NORMAL, "Erasing blocks: 0/%d (0%%)", blocks);

		for (i = 1; i <= blocks; i++) {
			log_printf(LOG_NORMAL, "\rErasing blocks: %d/%d (%d%%)", i, blocks,
					PERCENTAGE(i, blocks));

			if (ioctl(dev_fd, MEMERASE, &erase) < 0) {
				log_printf(LOG_NORMAL, "\n");
				log_printf(LOG_ERROR,
						"While erasing blocks 0x%.8x-0x%.8x on %s: %m\n",
						(unsigned int) erase.start,
						(unsigned int) (erase.start + erase.length), device);

				return EXIT_FAILURE;
			}
			erase.start += mtd.erasesize;
		}

		log_printf(LOG_NORMAL, "\rErasing blocks: %d/%d (100%%)\n", blocks,
				blocks);
	} else {
		/* if not, erase the whole chunk in one shot */
		if (ioctl(dev_fd, MEMERASE, &erase) < 0) {
			log_printf(LOG_ERROR,
					"While erasing blocks from 0x%.8x-0x%.8x on %s: %m\n",
					(unsigned int) erase.start,
					(unsigned int) (erase.start + erase.length), device);

			return EXIT_FAILURE;
		}
	}

	DEBUG("Erased %u / %luk bytes\n", erase.length, filestat.st_size);

	return EXIT_SUCCESS;
}

static int flash_write(int dev_fd, int fil_fd, off_t offset, const char *device,
		const char *filename)
{
	int i = 0;
	size_t size, written;
	ssize_t result;
	unsigned char src[BUFSIZE], dest[BUFSIZE];

	if (flags & FLAG_VERBOSE)
		log_printf(LOG_NORMAL, "Writing data: 0k/%lluk (0%%)",
				KB((unsigned long long )filestat.st_size));

	size = filestat.st_size;
	i = BUFSIZE;
	written = 0;

	/* if offset is greater than 0 */
	if (offset)
		if (safe_rewind_offset(dev_fd, device, offset) != EXIT_SUCCESS)
			return EXIT_FAILURE;

	while (size) {
		if (size < BUFSIZE)
			i = size;
		if (flags & FLAG_VERBOSE)
			log_printf(LOG_NORMAL, "\rWriting data: %dk/%lluk (%llu%%)",
					KB(written + i), KB((unsigned long long )filestat.st_size),
					PERCENTAGE(written + i,
							(unsigned long long )filestat.st_size));

		/* read from filename */
		safe_read(fil_fd, filename, src, i, flags & FLAG_VERBOSE);

		/* write to device */
		result = write(dev_fd, src, i);
		if (i != result) {
			if (flags & FLAG_VERBOSE)
				log_printf(LOG_NORMAL, "\n");
			if (result < 0) {
				log_printf(LOG_ERROR,
						"While writing data to 0x%.8lx-0x%.8lx on %s: %m\n",
						written, written + i, device);

				return EXIT_FAILURE;
			}

			log_printf(LOG_ERROR,
					"Short write count returned while writing to x%.8zx-0x%.8zx "
							"on %s: %zu/%llu bytes written to flash\n", written,
					written + i, device, written + result,
					(unsigned long long) filestat.st_size);

			return EXIT_FAILURE;
		}

		written += i;
		size -= i;
	}

	if (flags & FLAG_VERBOSE)
		log_printf(LOG_NORMAL, "\rWriting data: %lluk/%lluk (100%%)\n",
				KB((unsigned long long )filestat.st_size),
				KB((unsigned long long )filestat.st_size));

	DEBUG("Wrote %d / %lluk bytes\n", written,
			(unsigned long long)filestat.st_size);

	return EXIT_SUCCESS;
}

static int flash_verify(int dev_fd, int fil_fd, off_t offset,
		const char *device, const char *filename)
{
	int i = BUFSIZE;
	size_t size = 0;
	unsigned char src[BUFSIZE], dest[BUFSIZE];

	safe_rewind(fil_fd, filename);
	safe_rewind_offset(dev_fd, device, offset);
	size = filestat.st_size;

	size_t written = 0;

	if (flags & FLAG_VERBOSE)
		log_printf(LOG_NORMAL, "Verifying data: 0k/%lluk (0%%)",
				KB((unsigned long long )filestat.st_size));

	while (size) {
		if (size < BUFSIZE)
			i = size;
		if (flags & FLAG_VERBOSE)
			log_printf(LOG_NORMAL, "\rVerifying data: %luk/%lluk (%llu%%)",
					KB(written + i), KB((unsigned long long )filestat.st_size),
					PERCENTAGE(written + i,
							(unsigned long long )filestat.st_size));

		/* read from filename */
		safe_read(fil_fd, filename, src, i, flags & FLAG_VERBOSE);

		/* read from device */
		safe_read(dev_fd, device, dest, i, flags & FLAG_VERBOSE);

		/* compare buffers */
		if (memcmp(src, dest, i)) {
			log_printf(LOG_ERROR,
					"File does not seem to match flash data. First mismatch at "
							"0x%.8zx-0x%.8zx\n", written, written + i);
			return EXIT_FAILURE;
		}

		written += i;
		size -= i;
	}

	if (flags & FLAG_VERBOSE)
		log_printf(LOG_NORMAL, "\rVerifying data: %lluk/%lluk (100%%)\n",
				KB((unsigned long long )filestat.st_size),
				KB((unsigned long long )filestat.st_size));

	DEBUG("Verified %d / %lluk bytes\n", written,
			(unsigned long long)filestat.st_size);

	return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
	const char *filename = NULL, *device = NULL;
	off_t offset;

	for (;;) {
		int option_index = 0;
		static const char *short_options = "hvAV";
		static const struct option long_options[] = {
				{ "help", no_argument, 0, 'h' },
				{ "verbose", no_argument, 0, 'v' },
				{ "erase-all", no_argument, 0, 'A' },
				{ "version", no_argument, 0, 'V' },
				{ 0, 0, 0, 0 },
		};

		int c = getopt_long(argc, argv, short_options, long_options,
				&option_index);

		if (c == EOF)
			break;

		switch (c) {
		case 'h':
			flags |= FLAG_HELP;
			DEBUG("Got FLAG_HELP\n");
			break;
		case 'v':
			flags |= FLAG_VERBOSE;
			DEBUG("Got FLAG_VERBOSE\n");
			break;
		case 'A':
			flags |= FLAG_ERASE_ALL;
			DEBUG("Got FLAG_ERASE_ALL\n");
			break;
		case 'V':
			common_print_version();
			exit(EXIT_SUCCESS);
			break;
		default:
			DEBUG("Unknown parameter: %s\n", argv[option_index]);
			showusage(true);
		}
	}

	if (optind + 3 == argc) {
		flags |= FLAG_FILENAME;
		filename = argv[optind];
		DEBUG("Got filename: %s\n", filename);

		flags |= FLAG_DEVICE;
		device = argv[optind + 1];
		DEBUG("Got device: %s\n", device);

		offset = strtoul(argv[optind + 2], NULL, 16);
		DEBUG("Got offset: %s\n", offset);
	}

	if (optind + 2 == argc) {
		flags |= FLAG_FILENAME;
		filename = argv[optind];
		DEBUG("Got filename: %s\n", filename);

		flags |= FLAG_DEVICE;
		device = argv[optind + 1];
		DEBUG("Got device: %s\n", device);

		offset = 0;
	}

	if (flags & FLAG_HELP || device == NULL)
		showusage(flags != FLAG_HELP);

	atexit(cleanup);

	/* get some info about the flash device */
	dev_fd = safe_open(device, O_SYNC | O_RDWR);
	if (ioctl(dev_fd, MEMGETINFO, &mtd) < 0) {
		DEBUG("ioctl(): %m\n");
		log_printf(LOG_ERROR,
				"This doesn't seem to be a valid MTD flash device!\n");
		exit(EXIT_FAILURE);
	}

	/* get some info about the file we want to copy */
	fil_fd = safe_open(filename, O_RDONLY);
	if (fstat(fil_fd, &filestat) < 0) {
		log_printf(LOG_ERROR, "While trying to get the file status of %s: %m\n",
				filename);
		exit(EXIT_FAILURE);
	}

	/* does it fit into the device/partition? */
	if (filestat.st_size > mtd.size) {
		log_printf(LOG_ERROR, "%s won't fit into %s!\n", filename, device);
		exit(EXIT_FAILURE);
	}

	/* does offset is out of the mtd */
	if (offset > mtd.size) {
		log_printf(LOG_ERROR, "%s offset won't fit into %s!\n", offset, device);
		exit(EXIT_FAILURE);
	}

	/* Erase flash partition based on the input file size */
	if (flash_erase(dev_fd, offset, device) != EXIT_SUCCESS)
		exit(EXIT_FAILURE);

	/* Write the entire file to flash */
	if (flash_write(dev_fd, fil_fd, offset, device, filename) != EXIT_SUCCESS)
		exit(EXIT_FAILURE);

	/* Verify that flash == file data */
	if (flash_verify(dev_fd, fil_fd, offset, device, filename) != EXIT_SUCCESS)
		exit(EXIT_FAILURE);

	log_printf(LOG_NORMAL,"done");
	exit(EXIT_SUCCESS);
}
