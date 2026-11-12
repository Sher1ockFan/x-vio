/* Jetson GMSL capture. stdout: int64 LE mapped RAW ns, optional sequence, then gray pixels. */
#define _GNU_SOURCE
#define CAMERA_PROBE_PIXEL_ONLY
#include "../cam_imu_motion/camera.c"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#if !defined(__aarch64__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error This camera requires a little-endian Linux AArch64 Jetson.
#endif

static int xioctl(int fd, unsigned long op, void *arg) {
    int rc;
    do { rc = ioctl(fd, op, arg); } while (rc < 0 && errno == EINTR);
    return rc;
}

static int64_t raw_ns(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC_RAW, &t)) { perror("clock_gettime"); exit(1); }
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

/* Estimate RAW minus CNTVCT with the narrowest RAW bracket, freshly for each frame. */
static int64_t clock_offset(uint64_t hz) {
    int64_t shortest = INT64_MAX, best = 0;
    for (unsigned i = 0; i < 16; ++i) {
        uint64_t counter;
        int64_t start = raw_ns();
        __asm__ __volatile__("isb; mrs %0, cntvct_el0; isb" : "=r"(counter) :: "memory");
        int64_t end = raw_ns();
        int64_t span = end - start;
        if (span < shortest) {
            shortest = span;
            int64_t cnt_ns = (int64_t)((counter / hz) * 1000000000ULL + ((counter % hz) * 1000000000ULL) / hz);
            best = start + span / 2 - cnt_ns;
        }
    }
    if (shortest > 1000000) { fprintf(stderr, "No reliable host clock bracket.\n"); exit(1); }
    return best;
}

static int full_layout(const struct v4l2_pix_format *p) {
    return p->width == WIDTH && p->height == HEIGHT && p->pixelformat == V4L2_PIX_FMT_UYVY &&
           p->bytesperline == STRIDE && p->sizeimage == FULL_SIZE && p->field == V4L2_FIELD_NONE;
}

#ifdef ORB_HUB_CAPTURE
static int set_full_layout(int fd, struct v4l2_format *fmt) {
    struct v4l2_pix_format *p = &fmt->fmt.pix;
    p->width = WIDTH;
    p->height = HEIGHT;
    p->pixelformat = V4L2_PIX_FMT_UYVY;
    p->field = V4L2_FIELD_NONE;
    p->bytesperline = 0;
    p->sizeimage = 0;
    return xioctl(fd, VIDIOC_S_FMT, fmt);
}

static int request_30fps(int fd) {
    struct v4l2_streamparm parm = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    if (xioctl(fd, VIDIOC_G_PARM, &parm) < 0) {
        if (errno != EINVAL && errno != ENOTTY) return -1;
    } else if (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
        struct v4l2_fract *interval = &parm.parm.capture.timeperframe;
        interval->numerator = 1;
        interval->denominator = 30;
        if (xioctl(fd, VIDIOC_S_PARM, &parm) == 0) {
            fprintf(stderr, "GMSL requested_fps=30 driver_interval=%u/%u s; see delivered_hz for measured fps.\n",
                    interval->numerator, interval->denominator);
            return 0;
        }
        if (errno != EINVAL && errno != ENOTTY) return -1;
    }
    fprintf(stderr, "GMSL driver did not accept an explicit 30 fps interval; see delivered_hz for measured fps.\n");
    return 0;
}

#endif

int main(int argc, char **argv) {
    if (argc != 1) {
        fprintf(stderr, "Usage: %s\n", argv[0]);
        return 2;
    }
    /* Live and recording address the same physical camera on B. */
    const char *device = "/dev/gmsl/B";
    const char *card = "vi-output, pnd_gmsl_mc 10-003b";
    const char *bus = "platform:tegra-capture-vi:0";
    int fd = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) { perror(device); return 1; }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_capability cap = {0};
    struct v4l2_format fmt = {.type = type};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) || xioctl(fd, VIDIOC_G_FMT, &fmt)) { perror("inspect camera"); return 1; }
    const struct v4l2_pix_format *p = &fmt.fmt.pix;
    if (strcmp((char *)cap.driver, "tegra-video") || strcmp((char *)cap.card, card) ||
        strcmp((char *)cap.bus_info, bus)) {
        fprintf(stderr, "%s has unexpected GMSL identity; expected %s on %s.\n", device, card, bus);
        return 1;
    }
#ifdef ORB_HUB_CAPTURE
    int changing_layout = !full_layout(p);
    if (changing_layout && set_full_layout(fd, &fmt)) {
        perror("VIDIOC_S_FMT (close other camera readers)");
        return 1;
    }
#endif
    if (!full_layout(p)) {
        fprintf(stderr, "Expected %s GMSL camera, 1920x1441 UYVY stride 3840; check v4l2-ctl --all.\n", device);
        return 1;
    }
#ifdef ORB_HUB_CAPTURE
    if (changing_layout && request_30fps(fd)) { perror("set 30 fps camera interval"); return 1; }
    fprintf(stderr, "GMSL device=%s raw=%ux%u UYVY stride=%u crop_top=1 output=%ux%u mono8\n",
            device, p->width, p->height, p->bytesperline, p->width, p->height - 1);
#endif
    struct v4l2_requestbuffers req = {.count = 4, .type = type, .memory = V4L2_MEMORY_MMAP};
    if (xioctl(fd, VIDIOC_REQBUFS, &req)) { perror("VIDIOC_REQBUFS (close other camera readers)"); return 1; }
    if (req.count < 2 || req.count > 32) { fprintf(stderr, "Unexpected buffer count %u\n", req.count); return 1; }
    void *maps[32] = {0};
    for (unsigned i = 0; i < req.count; ++i) {
        struct v4l2_buffer b = {.type = type, .memory = V4L2_MEMORY_MMAP, .index = i};
        if (xioctl(fd, VIDIOC_QUERYBUF, &b)) { perror("VIDIOC_QUERYBUF"); return 1; }
        if (b.length < FULL_SIZE) { fprintf(stderr, "Short MMAP buffer.\n"); return 1; }
        maps[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (maps[i] == MAP_FAILED || xioctl(fd, VIDIOC_QBUF, &b)) { perror("map/queue camera buffer"); return 1; }
    }
    uint8_t *gray = malloc(GRAY_SIZE);
    uint64_t hz;
    __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(hz));
    if (!gray || !hz || hz > UINT32_MAX) { fprintf(stderr, "No gray buffer or invalid counter frequency.\n"); return 1; }
    FILE *timing_log = NULL;
    const char *timing_env = getenv("VIO_TIMING");
    if (timing_env && !strcmp(timing_env, "1")) {
        timing_log = fopen("camera_timing.csv", "wx");
        if (!timing_log) { perror("camera_timing.csv"); return 1; }
        setvbuf(timing_log, NULL, _IOLBF, 0);
        fprintf(timing_log, "sequence,driver_timestamp_ns,camera_raw_ns,dqbuf_raw_ns,gray_start_raw_ns,"
                "gray_done_raw_ns,write_start_raw_ns,write_end_raw_ns,bytesused,flags\n");
    }
    if (xioctl(fd, VIDIOC_STREAMON, &type)) { perror("VIDIOC_STREAMON"); return 1; }

    int64_t last = 0, first = 0;
    uint32_t sequence = 0, delivered = 0;
    uint64_t gaps = 0;
    struct pollfd wait = {.fd = fd, .events = POLLIN};
    for (;;) {
        int ready = poll(&wait, 1, 3000);
        if (ready < 0 && errno == EINTR) continue;
        if (ready <= 0 || !(wait.revents & POLLIN)) { fprintf(stderr, "Camera stalled, poll=%d revents=%d\n", ready, wait.revents); break; }
        struct v4l2_buffer b = {.type = type, .memory = V4L2_MEMORY_MMAP};
        if (xioctl(fd, VIDIOC_DQBUF, &b)) {
            if (errno == EAGAIN) continue;
            perror("VIDIOC_DQBUF"); break;
        }
        int64_t receipt = raw_ns();
        int64_t driver_stamp = (int64_t)b.timestamp.tv_sec * 1000000000LL + b.timestamp.tv_usec * 1000LL;
        int64_t event = driver_stamp + clock_offset(hz);
        int64_t lag = receipt - event;
        if (event <= last || lag < -5000000LL || lag > 250000000LL) {
            fprintf(stderr, "Unexpected camera clock/lag: event=%" PRId64 " lag_ms=%.2f. Recheck the driver clock domain.\n", event, (double)lag / 1e6);
            break;
        }
        int64_t gray_start = timing_log ? raw_ns() : 0;
        if (b.index >= req.count || (b.flags & V4L2_BUF_FLAG_ERROR) ||
            extract_gray(gray, maps[b.index], b.bytesused)) {
            fprintf(stderr, "Invalid camera frame at sequence %u.\n", b.sequence); break;
        }
        int64_t gray_done = timing_log ? raw_ns() : 0;
        struct v4l2_buffer q = {.type = type, .memory = V4L2_MEMORY_MMAP, .index = b.index};
        if (xioctl(fd, VIDIOC_QBUF, &q)) { perror("VIDIOC_QBUF"); break; }
        if (delivered) gaps += (uint32_t)(b.sequence - sequence - 1);
        else first = event;
        last = event;
        sequence = b.sequence;
        ++delivered;
        if (delivered == 1 || !(delivered % 150))
            fprintf(stderr, "GMSL frames=%u seq_missing=%" PRIu64 " lag_ms=%.2f delivered_hz=%.1f\n", delivered, gaps,
                    (double)lag / 1e6, delivered > 1 ? (delivered - 1) * 1e9 / (event - first) : 0.0);
        int64_t write_start = timing_log ? raw_ns() : 0;
        if (fwrite(&event, sizeof(event), 1, stdout) != 1 ||
#ifdef ORB_CAMERA_SEQUENCE
            fwrite(&sequence, sizeof(sequence), 1, stdout) != 1 ||
#endif
            fwrite(gray, 1, GRAY_SIZE, stdout) != GRAY_SIZE || fflush(stdout)) {
            perror("camera output pipe"); break;
        }
        if (timing_log) {
            int64_t write_end = raw_ns();
            if (fprintf(timing_log, "%u,%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64
                        ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%u,%u\n",
                        sequence, driver_stamp, event, receipt, gray_start, gray_done,
                        write_start, write_end, b.bytesused, b.flags) < 0) {
                perror("write camera timing"); break;
            }
        }
    }
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    close(fd);
    free(gray);
    if (timing_log) fclose(timing_log);
    return 1;  /* Normal trial end is signalled by the parent closing the capture worker. */
}
