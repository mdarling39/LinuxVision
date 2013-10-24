#ifndef V4L2CAP_H
#define V4L2CAP_H



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


class v4l2Cap
{
public:

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

private:


// Capture settings
    char                dev_name[15];
    io_method           io;
    int                 fd;
    struct buffer       *buffers;
    unsigned int        n_buffers;
    int                 out_buf;
    int                 frame_count;
    int                 set_format;
    unsigned int        pixel_format;
    unsigned int        width;
    unsigned int        height;
    unsigned int        fps;
    unsigned int        timeout;
    unsigned int        timeouts_max;
    char                *out_name;
public:
    bool                customInit;



public:

    v4l2Cap();
    int read_frame(cv::Mat&);
    void grab_frame(cv::Mat&);
    void close_device(void);
    void open_device(void);

    // Setters and getters
    void set_dev_name(char*);
    void set_pix_fmt(unsigned int);
    void set_width(unsigned int);
    void set_height(unsigned int);
    void set_fps(unsigned int);

private:

    void stop_capturing(void);
    void start_capturing(void);
    void uninit_device(void);
    void init_device(void);
    void custom_init(void);
    void process_image(const void *p, int size, cv::Mat&);
    void init_read(unsigned int buffer_size);
    void init_mmap(void);
    void init_userp(unsigned int buffer_size);


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

    void set_parm(uint32_t control_id, int32_t control_value)
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

};
#endif //V4L2CAP_H
