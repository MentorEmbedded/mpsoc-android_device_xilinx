#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/media.h>
#include <linux/types.h>
#include <linux/v4l2-mediabus.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <mediactl.h>
#include <mediactl-priv.h>
#include <v4l2subdev.h>

#include "xilinx-v4l2-controls.h"

#define MEDIA_TPG_ENTITY_1 "a00e0000.tpg"
#define MEDIA_TPG_ENTITY_2 "a02a0000.tpg1"

#define TPG_SUBDEV_NAME "a00e0000.tpg"

#define MEDIA_TPG_FMT_IN "VYYUYY8"
//#define MEDIA_TPG_FMT_IN "UYVY"
#define TPG_BG_PATTERN_DEFAULT 11
#define TPG_FG_DEFAULT 1
#define TPG_PPC_DEFAULT 1
#define TPG_4K_HOR_BLANKING 560
#define TPG_4K_VER_BLANKING 90

static unsigned int bg_pattern = TPG_BG_PATTERN_DEFAULT;
static unsigned int fg_pattern = TPG_FG_DEFAULT;
static unsigned int ppc_set = TPG_PPC_DEFAULT;

#define TPG_MEDIA_DEV "/dev/media2"

#define ASSERT2(cond, ...) 					\
		do {							\
			if (!(cond)) { 				\
				int errsv = errno;			\
				fprintf(stderr, "ERROR(%s:%d) : ",	\
						__FILE__, __LINE__);	\
				errno = errsv;				\
				fprintf(stderr,  __VA_ARGS__);		\
				abort();				\
			}						\
		} while(0)

void print_media_info(const struct media_device_info *info)
{
	printf("Media controller API version %u.%u.%u\n\n",
						   (info->media_version << 16) & 0xff,
						   (info->media_version << 8) & 0xff,
						   (info->media_version << 0) & 0xff);
	printf("Media device information\n"
						   "------------------------\n"
						   "driver			%s\n"
						   "model			%s\n"
						   "serial			%s\n"
						   "bus info		%s\n"
						   "hw revision 	0x%x\n"
						   "driver version	%u.%u.%u\n\n",
						   info->driver, info->model,
						   info->serial, info->bus_info,
						   info->hw_revision,
						   (info->driver_version << 16) & 0xff,
						   (info->driver_version << 8) & 0xff,
						   (info->driver_version << 0) & 0xff);
}

int v4l2_set_ctrl(struct media_entity *entity, int id, int value)
{
	int ret;
	struct v4l2_queryctrl query;
	struct v4l2_control ctrl;


	ret = v4l2_subdev_open(entity);
	ASSERT2(ret == 0, "failed to media_entity %s: %s\n", entity->devname, strerror(errno));

	memset(&query, 0, sizeof(query));
	query.id = id;
	ret = ioctl(entity->fd, VIDIOC_QUERYCTRL, &query);
	ASSERT2(ret >= 0, "VIDIOC_QUERYCTRL on %s failed: %s\n", entity->devname, strerror(errno));

	if (query.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf("V4L2_CID_%d is disabled\n", id);
	} else {
		memset(&ctrl, 0, sizeof(ctrl));
		ctrl.id = query.id;
		ctrl.value = value;
		ret = ioctl(entity->fd, VIDIOC_S_CTRL, &ctrl);
		ASSERT2(ret >= 0, "VIDIOC_S_CTRL failed: %s\n", strerror(errno));
	}

	v4l2_subdev_close(entity);
	return 0;
}


void tpg_set_blanking(struct media_entity *entity, unsigned int vblank,
		      unsigned int hblank)
{
	v4l2_set_ctrl(entity, V4L2_CID_VBLANK, vblank);
	v4l2_set_ctrl(entity, V4L2_CID_HBLANK, hblank);
}

void tpg_set_bg_pattern(struct media_entity *entity, unsigned int bg)
{
	v4l2_set_ctrl(entity, V4L2_CID_TEST_PATTERN, bg);
	bg_pattern = bg;
}

void tpg_set_fg_pattern(struct media_entity *entity, unsigned int fg)
{
	v4l2_set_ctrl(entity, V4L2_CID_XILINX_TPG_HLS_FG_PATTERN, fg);
	fg_pattern = fg;
}

void tpg_set_ppc(struct media_entity *entity, unsigned int ppc)
{
	v4l2_set_ctrl(entity, V4L2_CID_XILINX_PPC, ppc);
	 ppc_set = ppc;
}


static void tpg_set_cur_config(struct media_entity *entity)
{
	/* Set current TPG config */
	tpg_set_bg_pattern(entity, bg_pattern);
	/* Box overlay is disabled i.e no foreground pattern */
	tpg_set_fg_pattern(entity, fg_pattern);

	/* TODO: Need to fix this by giving hblank and vblank as per input resolution.
	 * It will be fixed when we add support to dynamically detect the native resolution
	 * of the monitor.
	 */
	tpg_set_blanking(entity, TPG_4K_HOR_BLANKING, TPG_4K_VER_BLANKING);
}

struct media_entity* find_tpg_subdev(struct media_device *media)
{
	size_t nents = media_get_entities_count(media);
	for (size_t i = 0; i < nents; i++) {
		struct media_entity *entity = media_get_entity(media, i);

		if (!entity) {
			printf("failed to get entity %zu\n", i);
			continue;
		}
		const struct media_entity_desc *info;
		info = media_entity_get_info(entity);

		if (info && 
			!strncmp(info->name, TPG_SUBDEV_NAME, strlen(TPG_SUBDEV_NAME))){
			return entity;
		}
	}
	return NULL;
}


#define MEDIA_FMT "\"%s\":%d [fmt:%s/%dx%d field:none]"

void media_set_fmt_str(char *set_fmt, char const * entity, unsigned int pad,
		       const char *fmt, unsigned int width, unsigned int height)
{
	sprintf(set_fmt, MEDIA_FMT, entity, pad, fmt, width, height);
	printf("set_fmt = %s\n", set_fmt);
}

int main()
{
	int ret;
	char fmt_str[100];
	struct media_device *media = media_device_new(TPG_MEDIA_DEV);

	if (!media) {
		perror("Failed to create media device");
		return -1;
	}

	ret = media_device_enumerate(media);
	if (ret < 0) {
		perror("Failed to enumerate media device");
		media_device_unref(media);
		return -1;
	}

		media_debug_set_handler(media,
			(void (*)(void *, ...))fprintf, stdout);

	const struct media_device_info *info = media_get_info(media);
	if (info)
		print_media_info(info);
	
	struct media_entity *tpg_entity = find_tpg_subdev(media);
	if (!tpg_entity){
		perror("Failed to find TPG subdev");
		media_device_unref(media);
		return -1;
	}
	printf("Setting TPG subdev config ...\n");
	tpg_set_cur_config(tpg_entity);


	const struct media_entity_desc *ent_info;
	ent_info = media_entity_get_info(tpg_entity);

	/* Set TPG input resolution */
	memset(fmt_str, 0, sizeof (fmt_str));
	media_set_fmt_str(fmt_str, ent_info->name, 0, MEDIA_TPG_FMT_IN, 3840, 2160);
//	media_set_fmt_str(fmt_str, ent_info->name, 0, MEDIA_TPG_FMT_IN, 1920, 1080);
	ret = v4l2_subdev_parse_setup_formats(media, fmt_str);
	if (ret){
		perror("Unable to setup formats");
		media_device_unref(media);
		return -1;	
	}

	// int fd;

	// fd = open("/dev/v4l-subdev2", O_RDWR);
	// if (fd == -1){
	// 	perror("Opening video device");
	// 	return 1;
	// }
	// // if(print_caps(fd))
	// // return 1;

	// tpg_set_cur_config(fd);
	// // if(init_mmap(fd))
	// //     return 1;
	// // int i;
	// // for(i=0; i<1; i++)
	// // {
	// //     if(capture_image(fd))
	// //         return 1;
	// // }
	// close(fd);
	media_device_unref(media);
	return 0;
}
