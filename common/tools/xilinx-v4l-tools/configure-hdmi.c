#include <stdlib.h>
#include <mediactl.h>
#include <v4l2subdev.h>
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
#include <mediactl-priv.h>

#define MEDIA_HDMI_ENTITY_1	"a0000000.hdmi_rx_ss"
#define MEDIA_HDMI_ENTITY_2	"a02e0000.dummy"
#define MEDIA_HDMI_ENTITY_3	"a02d0000.dummy_2"
#define MEDIA_SCALER_ENTITY	"a0080000.scaler"
#define VCAP_HDMI_HAS_SCALER 1
#define MEDIA_SCALER_FMT_OUT	"VYYUYY8"
//#define MEDIA_SCALER_FMT_OUT	"YUV"
#define MEDIA_HDMI_PAD		0
#define VCAP_HDMI_FLAG_HAS_SCALER	BIT(0)

#define HDMI_MEDIA_DEV "/dev/media4"

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

static int width_in;
static int height_in;

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

struct media_entity* find_hdmi_subdev(struct media_device *media)
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
			!strncmp(info->name, MEDIA_HDMI_ENTITY_1, strlen(MEDIA_HDMI_ENTITY_1))){
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

#define MEDIA_PAD "\"%s\":%d"
void media_set_pad_str(char *set_fmt, char const * entity, unsigned int pad)
{
	sprintf(set_fmt, MEDIA_PAD, entity, pad);
}

int main()
{
	int ret;
	char fmt_str[100];
	struct media_device *media = media_device_new(HDMI_MEDIA_DEV);
	struct v4l2_mbus_framefmt format;
	struct media_pad *pad;
	struct v4l2_dv_timings timings;
	const char* fmt_code;

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
	
	struct media_entity *hdmi_entity = find_hdmi_subdev(media);
	if (!hdmi_entity){
		perror("Failed to find HDMI subdev");
		media_device_unref(media);
		return -1;
	}
	printf("Setting HDMI subdev config ...\n");
	//tpg_set_cur_config(tpg_entity);


	const struct media_entity_desc *ent_info;
	ent_info = media_entity_get_info(hdmi_entity);

	/* Get HDMI Rx pad */
	memset(fmt_str, 0, sizeof (fmt_str));
	media_set_pad_str(fmt_str, ent_info->name, MEDIA_HDMI_PAD);
	pad = media_parse_pad(media, fmt_str, NULL);
	if(!pad){
		perror("Pad not found");
		media_device_unref(media);
		return -1;
	}

	ret = v4l2_subdev_query_dv_timings(pad->entity, &timings);
	if (ret < 0 ) {
		/* Delay dv_timings query in-case of failure */
		perror("Failed to query DV timings");
		media_device_unref(media);
		return -1;
	}

	/* Retrieve HDMI Rx pad format */
	ret = v4l2_subdev_get_format(pad->entity, &format, MEDIA_HDMI_PAD,
				     V4L2_SUBDEV_FORMAT_ACTIVE);
	if (ret) {
		perror("Failed to get HDMI Rx pad format");
		media_device_unref(media);
		return -1;
	}

	fmt_code = v4l2_subdev_pixelcode_to_string(format.code);
	printf("HDMI Rx source pad format: %s, %ux%u\n", fmt_code,
		 format.width, format.height);

	/* Set Scaler resolution */
	memset(fmt_str, 0, sizeof(fmt_str));
	
	media_set_fmt_str(fmt_str, MEDIA_SCALER_ENTITY, 0, fmt_code,
				timings.bt.width, timings.bt.height);

	width_in = timings.bt.width;
	height_in = timings.bt.height;

	ret = v4l2_subdev_parse_setup_formats(media, fmt_str);
	if (ret) {
		perror("Unable to setup formats");
		media_device_unref(media);
		return -1;
	}

	memset(fmt_str, 0, sizeof(fmt_str));
	media_set_fmt_str(fmt_str, MEDIA_SCALER_ENTITY, 1,
				  MEDIA_SCALER_FMT_OUT,
				  timings.bt.width,
				  timings.bt.height);

	ret = v4l2_subdev_parse_setup_formats(media, fmt_str);
	if (ret) {
		perror("Unable to setup formats");
		media_device_unref(media);
		return -1;
	}

	media_device_unref(media);
	return 0;
}
