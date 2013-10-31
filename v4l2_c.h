/*  This file uses a C interface for capture from Video4Linux2 devices.
 *  It contains some mutex variables that must be declared globally.
 *  Assumes using memory mapping.
 */

#ifndef V4L2_C_H_INCLUDED
#define V4L2_C_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <iostream>

#include <linux/videodev2.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/opencv.hpp"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

using namespace std;

// Helper functions
int xioctl(int fh, int request, void *arg);
void errno_exit(const char *s);

enum io_method
{
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

struct buffer
{
    void *start;
    size_t length;
};

struct v4l2Parms
{
    char                dev_name[15];
    int                 fd;
    struct buffer       *buffers;
    unsigned int        n_buffers;
    int                 frame_count;
    unsigned int        pixel_format;
    unsigned int        width;
    unsigned int        height;
    unsigned int        fps;
    unsigned int        timeout;
    void                (*customInitFcn)(void*);
};


static void v4l2_set_defalut_parms(struct v4l2Parms* parm)
{
    // Default capture settings
    strcpy(parm->dev_name,"/dev/video0");
    parm->fd = -1;
    parm->frame_count = 1;
    parm->pixel_format = V4L2_PIX_FMT_MJPEG;
    parm->width = 640;
    parm->height = 480;
    parm->fps = 30;
    parm->timeout = 1;
    parm->customInitFcn = NULL;
};


static void v4l2_process_image(cv::Mat &img, const void *p)
{
        cv::Mat buff(img.cols, img.rows, CV_8UC3, (void*)p);
        img = cv::imdecode(buff,CV_LOAD_IMAGE_COLOR);
}


static int v4l2_fill_buffer(struct v4l2Parms* parm, struct v4l2_buffer* buf, void **buffer_ptr)
{

    CLEAR(*buf);

    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(parm->fd, VIDIOC_DQBUF, buf))
    {
        switch (errno)
        {
        case EAGAIN:
            return 0;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
            errno_exit("VIDIOC_DQBUF");
        }
    }

    assert(buf->index < parm->n_buffers);

    *buffer_ptr = parm->buffers[buf->index].start;

    return 1;
}

static int v4l2_queue_buffer(struct v4l2Parms* parm, struct v4l2_buffer* buf)
{
    if (-1 == xioctl(parm->fd, VIDIOC_QBUF, buf))
        errno_exit("VIDIOC_QBUF");
    return 0;
}


void v4l2_grab_frame(struct v4l2Parms* parm,cv::Mat &img)
{
   struct v4l2_buffer buf;
    void* buff_ptr;

    v4l2_fill_buffer(parm, &buf, &buff_ptr); // dequeue buffer
    v4l2_process_image(img, buff_ptr);
    v4l2_queue_buffer(parm, &buf);
}

static int v4l2_wait_for_data(struct v4l2Parms* parm)
{
    for (;;)
    {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(parm->fd, &fds);

        /* Timeout. */
        tv.tv_sec = parm->timeout;
        tv.tv_usec = 0;

        r = select(parm->fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno)
                continue;
            return -1;
        }

        if (0 == r)
        {
            fprintf(stderr, "select timeout\n");
            return -1;
        }

        return 0;
    }
}


void v4l2_stop_capturing(struct v4l2Parms *parm)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(parm->fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");
}

static void v4l2_start_capturing(struct v4l2Parms* parm)
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < parm->n_buffers; ++i)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(parm->fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(parm->fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");
}

void v4l2_uninit_device(struct v4l2Parms* parm)
{
    unsigned int i;
        for (i = 0; i < parm->n_buffers; ++i)
            if (-1 == munmap(parm->buffers[i].start, parm->buffers[i].length))
                errno_exit("munmap");

    free(parm->buffers);
}



static void v4l2_init_mmap(struct v4l2Parms* parm)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(parm->fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s does not support "
                    "memory mapping\n", parm->dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2)
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n",
                parm->dev_name);
        exit(EXIT_FAILURE);
    }

    (parm->buffers) = (buffer*)calloc(req.count, sizeof (*(parm->buffers)));

    if (!(parm->buffers))
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (parm->n_buffers = 0; parm->n_buffers < req.count; ++(parm->n_buffers))
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = parm->n_buffers;

        if (-1 == xioctl(parm->fd, VIDIOC_QUERYBUF, &buf))
            errno_exit("VIDIOC_QUERYBUF");

        parm->buffers[parm->n_buffers].length = buf.length;
        parm->buffers[parm->n_buffers].start =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 parm->fd, buf.m.offset);

        if (MAP_FAILED == parm->buffers[parm->n_buffers].start)
            errno_exit("mmap");
    }
}

static void v4l2_init_device(struct v4l2Parms* parm)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    struct v4l2_streamparm frameint;

    unsigned int min;

    if (-1 == xioctl(parm->fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s is no V4L2 device\n",
                    parm->dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                parm->dev_name);
        exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                parm->dev_name);
        exit(EXIT_FAILURE);
    }


    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(parm->fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(parm->fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = parm->width;
    fmt.fmt.pix.height = parm->height;
    fmt.fmt.pix.pixelformat = parm->pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (-1 == xioctl(parm->fd, VIDIOC_S_FMT, &fmt))
        errno_exit("VIDIOC_S_FMT");

    if (fmt.fmt.pix.pixelformat != parm->pixel_format)
    {
        fprintf(stderr,"Libv4l didn't accept pixel format. Can't proceed.\n");
        exit(EXIT_FAILURE);
    }


    CLEAR(frameint);

    /* Attempt to set the frame interval. */
    frameint.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frameint.parm.capture.timeperframe.numerator = 1;
    frameint.parm.capture.timeperframe.denominator = parm->fps;
    if (-1 == xioctl(parm->fd, VIDIOC_S_PARM, &frameint))
        fprintf(stderr, "Unable to set frame interval.\n");

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    /* Initialize memory mapping */
    v4l2_init_mmap(parm);

    if (parm->customInitFcn != NULL)
            parm->customInitFcn((void*)parm);

}

void v4l2_close_device(struct v4l2Parms* parm)
{

    if (-1 == close(parm->fd))
        errno_exit("close");

    parm->fd = -1;
}

static void v4l2_open_device(struct v4l2Parms* parm)
{
    struct stat st;

    if (-1 == stat(parm->dev_name, &st))
    {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                parm->dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode))
    {
        fprintf(stderr, "%s is no device\n", parm->dev_name);
        exit(EXIT_FAILURE);
    }

    parm->fd = open(parm->dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == parm->fd)
    {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                parm->dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}


// Helper functions
    int xioctl(int fh, int request, void *arg)
    {
        int r;

        do
        {
            r = ioctl(fh, request, arg);
        }
        while (-1 == r && EINTR == errno);

        return r;
    }

    void errno_exit(const char *s)
    {
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    void set_parm(int fd, uint32_t control_id, int32_t control_value)
    {
        struct v4l2_control control;
        control.id = control_id;
        control.value = control_value;

        char msg[200];
        sprintf(msg,"Could not modify v4l2_control value given by flag: %X", control_id);
        if (0 != xioctl(fd,VIDIOC_S_CTRL,&control))
        {
            errno_exit(msg);
        }
    }

    void set_manual_exposure(int fd, int32_t control_value)
    {
        struct v4l2_ext_control control[2];
        control[0].id = V4L2_CID_EXPOSURE_AUTO;
        control[0].value = V4L2_EXPOSURE_MANUAL;
        control[1].id = V4L2_CID_EXPOSURE_ABSOLUTE;
        control[1].value = control_value;

        struct v4l2_ext_controls controls;
        controls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
        controls.count = 2;
        controls.controls = control;

        char msg[200];
        sprintf(msg,"Could not set manual exposure");
        if (0 != xioctl(fd,VIDIOC_S_EXT_CTRLS,&controls))
        {
            errno_exit(msg);
        }
    }

    void set_auto_exposure(int fd)
        {
        struct v4l2_ext_control control[2];
        control[0].id = V4L2_CID_EXPOSURE_AUTO;
        control[0].value = V4L2_EXPOSURE_APERTURE_PRIORITY;
        control[1].id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
        control[1].value = false;


        struct v4l2_ext_controls controls;
        controls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
        controls.count = 2;
        controls.controls = control;

        char msg[200];
        sprintf(msg,"Could not set auto exposure");
        if (0 != xioctl(fd,VIDIOC_S_EXT_CTRLS,&controls))
        {
            errno_exit(msg);
        }
    }

#endif // V4L2_C_H_INCLUDED
