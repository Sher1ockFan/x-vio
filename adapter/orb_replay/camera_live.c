/* Jetson UYVY capture. stdout: int64 LE RAW ns, optional uint32 sequence, gray. */
#define _GNU_SOURCE
#include "gray.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#if !defined(__linux__) || !defined(__aarch64__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error This camera requires a little-endian Linux AArch64 Jetson.
#endif

enum camera_clock { CAMERA_CNTVCT, CAMERA_MONOTONIC, CAMERA_RAW };
#define TEGRA_FRAME_RATE_ID UINT32_C(0x009a200b)
#define TEGRA_FRAME_RATE_CONTROL "tegra_frame_rate_microhz"
struct camera_profile {
    const char *device, *card, *bus, *clock_name;
    unsigned width, height, top, bottom, fps;
    enum camera_clock clock;
    int use_tegra_frame_rate_control;
};
struct mapping { void *data; size_t length; };
static volatile sig_atomic_t stopped;
static void stop_capture(int signum) { stopped = signum; }

static int xioctl(int fd, unsigned long op, void *arg) {
    int rc;
    do { rc = ioctl(fd, op, arg); } while (rc < 0 && errno == EINTR && !stopped);
    return rc;
}

static const char *required_text(const char *name) {
    const char *value = getenv(name);
    if (!value || !*value) fprintf(stderr, "Missing required camera profile value %s.\n", name);
    return value && *value ? value : NULL;
}

static int required_uint(const char *name, unsigned maximum, unsigned *out) {
    const char *value = required_text(name);
    if (!value) return -1;
    unsigned result = 0;
    for (const char *p = value; *p; ++p) {
        if (*p < '0' || *p > '9' || result > maximum / 10u ||
            (result == maximum / 10u && (unsigned)(*p - '0') > maximum % 10u)) {
            fprintf(stderr, "%s must be an unsigned decimal integer <= %u.\n", name, maximum);
            return -1;
        }
        result = result * 10u + (unsigned)(*p - '0');
    }
    *out = result;
    return 0;
}

static int load_profile(struct camera_profile *c) {
    c->device = required_text("VIO_CAMERA_DEVICE");
    c->card = required_text("VIO_CAMERA_CARD");
    c->clock_name = required_text("VIO_CAMERA_CLOCK");
    c->bus = getenv("VIO_CAMERA_BUS");
    if (c->bus && !*c->bus) c->bus = NULL;
    const char *fps_control = getenv("VIO_CAMERA_FPS_CONTROL");
    if (fps_control) {
        if (strcmp(fps_control, TEGRA_FRAME_RATE_CONTROL)) {
            fprintf(stderr, "Unknown VIO_CAMERA_FPS_CONTROL: %s.\n", fps_control);
            return -1;
        }
        c->use_tegra_frame_rate_control = 1;
    }
    if (!c->device || !c->card || !c->clock_name ||
        required_uint("VIO_CAMERA_WIDTH", 16384, &c->width) ||
        required_uint("VIO_CAMERA_HEIGHT", 16384, &c->height) ||
        required_uint("VIO_CAMERA_CROP_TOP", 16384, &c->top) ||
        required_uint("VIO_CAMERA_CROP_BOTTOM", 16384, &c->bottom) ||
        required_uint("VIO_CAMERA_FPS", 240, &c->fps)) return -1;
    if (c->device[0] != '/' || strlen(c->card) >= sizeof(((struct v4l2_capability *)0)->card) ||
        (c->bus && strlen(c->bus) >= sizeof(((struct v4l2_capability *)0)->bus_info))) {
        fprintf(stderr, "Camera device must be absolute; card/bus must fit V4L2 identity fields.\n");
        return -1;
    }
    struct gray_layout layout;
    if (!c->fps || gray_layout_init(&layout, c->width, c->height,
                                    (size_t)c->width * 2u, c->top, c->bottom)) {
        fprintf(stderr, "Invalid camera dimensions, crop, or fps; output must contain at least one row.\n");
        return -1;
    }
    if (!strcmp(c->clock_name, "cntvct")) c->clock = CAMERA_CNTVCT;
    else if (!strcmp(c->clock_name, "monotonic")) c->clock = CAMERA_MONOTONIC;
    else if (!strcmp(c->clock_name, "raw")) c->clock = CAMERA_RAW;
    else {
        fprintf(stderr, "VIO_CAMERA_CLOCK must explicitly select cntvct, monotonic, or raw.\n");
        return -1;
    }
    return 0;
}

static void json_string(const char *value) {
    putchar('"');
    for (const unsigned char *p = (const unsigned char *)value; *p; ++p) {
        if (*p == '"' || *p == '\\') printf("\\%c", *p);
        else if (*p < 32 || *p >= 127) printf("\\u%04x", *p);
        else putchar(*p);
    }
    putchar('"');
}

static void describe(const struct camera_profile *c) {
    printf("{\"schema_version\":1,\"device\":"); json_string(c->device);
    printf(",\"driver\":\"tegra-video\",\"card\":"); json_string(c->card);
    printf(",\"bus\":"); if (c->bus) json_string(c->bus); else printf("null");
    printf(",\"pixel_format\":\"UYVY\",\"requested_width\":%u,\"requested_height\":%u,"
           "\"crop_top\":%u,\"crop_bottom\":%u,\"output_width\":%u,\"output_height\":%u,"
           "\"requested_fps\":%u,\"clock_domain\":", c->width, c->height, c->top, c->bottom,
           c->width, c->height - c->top - c->bottom, c->fps);
    json_string(c->clock_name);
    if (c->use_tegra_frame_rate_control)
        printf(",\"frame_rate_control\":\"" TEGRA_FRAME_RATE_CONTROL "\"");
    printf("}\n");
}

static int64_t clock_ns(clockid_t clock) {
    struct timespec t;
    if (clock_gettime(clock, &t)) { perror("clock_gettime"); exit(1); }
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static int64_t raw_ns(void) { return clock_ns(CLOCK_MONOTONIC_RAW); }

/* The profile selects the verified driver domain. Never infer it from flags or
 * lag. Re-estimate RAW-minus-domain for every frame, preserving the old method. */
static int clock_offset(enum camera_clock domain, uint64_t hz, int64_t *offset) {
    if (domain == CAMERA_RAW) { *offset = 0; return 0; }
    int64_t shortest = INT64_MAX, best = 0;
    for (unsigned i = 0; i < 16; ++i) {
        int64_t value;
        int64_t start = raw_ns();
        if (domain == CAMERA_CNTVCT) {
            uint64_t counter;
            __asm__ __volatile__("isb; mrs %0, cntvct_el0; isb" : "=r"(counter) :: "memory");
            value = (int64_t)((counter / hz) * 1000000000ULL +
                             ((counter % hz) * 1000000000ULL) / hz);
        } else value = clock_ns(CLOCK_MONOTONIC);
        int64_t end = raw_ns(), span = end - start;
        if (span >= 0 && span < shortest) {
            shortest = span;
            best = start + span / 2 - value;
        }
    }
    if (shortest > 1000000) { fprintf(stderr, "No reliable host clock bracket.\n"); return -1; }
    *offset = best;
    return 0;
}

static int format_matches(const struct v4l2_pix_format *p, const struct camera_profile *c) {
    return p->width == c->width && p->height == c->height &&
           p->pixelformat == V4L2_PIX_FMT_UYVY && p->field == V4L2_FIELD_NONE;
}

static int request_fps(int fd, unsigned fps) {
    /* Always try S_PARM, even when the existing dimensions match. */
    struct v4l2_streamparm request = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    request.parm.capture.timeperframe = (struct v4l2_fract){1, fps};
    int accepted = xioctl(fd, VIDIOC_S_PARM, &request) == 0;
    if (!accepted && errno != EINVAL && errno != ENOTTY && errno != EOPNOTSUPP) return -1;
    struct v4l2_streamparm actual = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    int reported = xioctl(fd, VIDIOC_G_PARM, &actual) == 0;
    if (!reported && errno != EINVAL && errno != ENOTTY && errno != EOPNOTSUPP) return -1;
    const struct v4l2_fract *interval = reported ? &actual.parm.capture.timeperframe :
                                                &request.parm.capture.timeperframe;
    fprintf(stderr, "GMSL requested_fps=%u interval_request=%s ", fps,
            accepted ? "accepted" : "unsupported");
    if ((reported || accepted) && interval->numerator && interval->denominator)
        fprintf(stderr, "driver_interval=%u/%u s reported_fps=%.6f%s; ",
                interval->numerator, interval->denominator,
                (double)interval->denominator / interval->numerator,
                reported ? "" : " (S_PARM return)");
    else fprintf(stderr, "reported_fps=unknown; ");
    fprintf(stderr, "see delivered_hz for measured fps.\n");
    return 0;
}

/* The jl_gmsl calibration capture uses Tegra's frame_rate control in microhertz.
 * S_PARM alone can leave this sensor at 30 fps. Apply this after S_PARM. */
static int set_tegra_frame_rate(int fd, unsigned fps) {
    int64_t target = (int64_t)fps * 1000000;
    struct v4l2_query_ext_ctrl query = {.id = TEGRA_FRAME_RATE_ID};
    if (xioctl(fd, VIDIOC_QUERY_EXT_CTRL, &query)) {
        perror("VIDIOC_QUERY_EXT_CTRL frame_rate");
        return -1;
    }
    query.name[sizeof(query.name) - 1] = 0;
    char name[sizeof(query.name)];
    for (size_t i = 0; i < sizeof(name); ++i) {
        unsigned char ch = query.name[i];
        if (ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
        if (ch == ' ' || ch == '-') ch = '_';
        name[i] = (char)ch;
        if (!ch) break;
    }
    if (query.id != TEGRA_FRAME_RATE_ID || strcmp(name, "frame_rate") ||
        query.type != V4L2_CTRL_TYPE_INTEGER64 || query.minimum > target ||
        query.maximum < target || !query.step || ((uint64_t)target - (uint64_t)query.minimum) % query.step ||
        query.flags & (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE)) {
        fprintf(stderr, "Unexpected Tegra frame_rate control (id=0x%08x name=%s type=%u "
                        "min=%" PRId64 " max=%" PRId64 " step=%" PRIu64 " flags=0x%08x); "
                        "cannot request %u fps.\n",
                query.id, name, query.type, (int64_t)query.minimum, (int64_t)query.maximum,
                (uint64_t)query.step, query.flags, fps);
        return -1;
    }
    struct v4l2_ext_control item = {.id = TEGRA_FRAME_RATE_ID, .value64 = target};
    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CTRL_ID2CLASS(TEGRA_FRAME_RATE_ID), .count = 1, .controls = &item
    };
    if (xioctl(fd, VIDIOC_S_EXT_CTRLS, &controls)) {
        perror("VIDIOC_S_EXT_CTRLS frame_rate");
        return -1;
    }
    item.value64 = 0;
    if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &controls)) {
        perror("VIDIOC_G_EXT_CTRLS frame_rate");
        return -1;
    }
    if (item.value64 != target) {
        fprintf(stderr, "Tegra frame_rate readback=%" PRId64 "; expected=%" PRId64 " microhertz.\n",
                (int64_t)item.value64, target);
        return -1;
    }
    fprintf(stderr, "GMSL frame_rate_control=0x%08x requested_microhz=%" PRId64
                    " readback_microhz=%" PRId64 "; see delivered_hz for measured fps.\n",
            TEGRA_FRAME_RATE_ID, target, (int64_t)item.value64);
    return 0;
}

int main(int argc, char **argv) {
    int describe_only = argc == 2 && !strcmp(argv[1], "--describe");
    if (argc != 1 && !describe_only) { fprintf(stderr, "Usage: %s [--describe]\n", argv[0]); return 2; }
    struct camera_profile config = {0};
    if (load_profile(&config)) return 2;
    if (describe_only) { describe(&config); return ferror(stdout) ? 1 : 0; }

    struct sigaction action = {0};
    action.sa_handler = stop_capture;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) || sigaction(SIGTERM, &action, NULL)) { perror("sigaction"); return 1; }
    signal(SIGPIPE, SIG_IGN);
    int fd = -1, streaming = 0;
    unsigned mapped = 0;
    struct mapping maps[32] = {{0}};
    uint8_t *gray = NULL;
    FILE *timing_log = NULL;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_capability cap = {0};
    struct v4l2_format fmt = {.type = type};
    struct gray_layout layout;
    fd = open(config.device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) { perror(config.device); goto cleanup; }
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap)) { perror("inspect camera"); goto cleanup; }
    cap.driver[sizeof(cap.driver)-1] = cap.card[sizeof(cap.card)-1] = cap.bus_info[sizeof(cap.bus_info)-1] = 0;
    if (strcmp((char *)cap.driver, "tegra-video") || strcmp((char *)cap.card, config.card) ||
        (config.bus && strcmp((char *)cap.bus_info, config.bus))) {
        fprintf(stderr, "%s has unexpected GMSL identity: driver=%s card=%s bus=%s.\n",
                config.device, cap.driver, cap.card, cap.bus_info);
        goto cleanup;
    }
    uint32_t caps = cap.capabilities & V4L2_CAP_DEVICE_CAPS ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_STREAMING) || !(caps & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "Camera lacks single-plane video capture or streaming.\n"); goto cleanup;
    }
    if (xioctl(fd, VIDIOC_G_FMT, &fmt)) { perror("inspect camera format"); goto cleanup; }
    if (config.use_tegra_frame_rate_control || !format_matches(&fmt.fmt.pix, &config)) {
        memset(&fmt.fmt.pix, 0, sizeof(fmt.fmt.pix));
        fmt.fmt.pix.width = config.width;
        fmt.fmt.pix.height = config.height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(fd, VIDIOC_S_FMT, &fmt)) { perror("VIDIOC_S_FMT (close other camera readers)"); goto cleanup; }
    }
    const struct v4l2_pix_format *p = &fmt.fmt.pix;
    if (!format_matches(p, &config) ||
        gray_layout_init(&layout, p->width, p->height, p->bytesperline, config.top, config.bottom) ||
        p->sizeimage < layout.minimum_source_bytes) {
        fprintf(stderr, "Unexpected camera format: requested=%ux%u UYVY; actual=%ux%u fourcc=0x%08x stride=%u size=%u field=%u.\n",
                config.width, config.height, p->width, p->height, p->pixelformat,
                p->bytesperline, p->sizeimage, p->field);
        goto cleanup;
    }
    if (request_fps(fd, config.fps)) { perror("set camera fps interval"); goto cleanup; }
    if (config.use_tegra_frame_rate_control && set_tegra_frame_rate(fd, config.fps)) goto cleanup;
    fprintf(stderr, "GMSL device=%s card=%s bus=%s raw=%ux%u UYVY stride=%u size=%u crop_top=%u crop_bottom=%u output=%zux%zu mono8 clock=%s\n",
            config.device, cap.card, cap.bus_info, p->width, p->height, p->bytesperline, p->sizeimage,
            config.top, config.bottom, layout.width, layout.height, config.clock_name);
    struct v4l2_requestbuffers req = {.count = 4, .type = type, .memory = V4L2_MEMORY_MMAP};
    if (xioctl(fd, VIDIOC_REQBUFS, &req)) { perror("VIDIOC_REQBUFS (close other camera readers)"); goto cleanup; }
    if (req.count < 2 || req.count > 32) { fprintf(stderr, "Unexpected buffer count %u\n", req.count); goto cleanup; }
    for (unsigned i = 0; i < req.count; ++i) {
        struct v4l2_buffer b = {.type = type, .memory = V4L2_MEMORY_MMAP, .index = i};
        if (xioctl(fd, VIDIOC_QUERYBUF, &b)) { perror("VIDIOC_QUERYBUF"); goto cleanup; }
        if (b.length < p->sizeimage) { fprintf(stderr, "Short MMAP buffer.\n"); goto cleanup; }
        void *data = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (data == MAP_FAILED) { perror("map camera buffer"); goto cleanup; }
        maps[i] = (struct mapping){data, b.length};
        ++mapped;
        if (xioctl(fd, VIDIOC_QBUF, &b)) { perror("queue camera buffer"); goto cleanup; }
    }
    gray = malloc(layout.pixels);
    uint64_t hz = 0;
    if (config.clock == CAMERA_CNTVCT) __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(hz));
    if (!gray || (config.clock == CAMERA_CNTVCT && (!hz || hz > UINT32_MAX))) {
        fprintf(stderr, "No gray buffer or invalid counter frequency.\n"); goto cleanup;
    }
    const char *timing_env = getenv("VIO_TIMING");
    if (timing_env && !strcmp(timing_env, "1")) {
        timing_log = fopen("camera_timing.csv", "wx");
        if (!timing_log) { perror("camera_timing.csv"); goto cleanup; }
        setvbuf(timing_log, NULL, _IOLBF, 0);
        if (fprintf(timing_log, "sequence,driver_timestamp_ns,camera_raw_ns,dqbuf_raw_ns,gray_start_raw_ns,"
                    "gray_done_raw_ns,write_start_raw_ns,write_end_raw_ns,bytesused,flags\n") < 0) goto cleanup;
    }
    if (stopped) goto cleanup;
    if (xioctl(fd, VIDIOC_STREAMON, &type)) { perror("VIDIOC_STREAMON"); goto cleanup; }
    streaming = 1;
    int64_t last = 0, first = 0;
    uint32_t sequence = 0;
    uint64_t gaps = 0, delivered = 0;
    struct pollfd wait = {.fd = fd, .events = POLLIN};
    while (!stopped) {
        int ready = poll(&wait, 1, 3000);
        if (ready < 0 && errno == EINTR) continue;
        if (ready <= 0 || !(wait.revents & POLLIN) || (wait.revents & (POLLNVAL | POLLHUP))) {
            fprintf(stderr, "Camera stalled, poll=%d revents=%d\n", ready, wait.revents); break;
        }
        struct v4l2_buffer b = {.type = type, .memory = V4L2_MEMORY_MMAP};
        if (xioctl(fd, VIDIOC_DQBUF, &b)) {
            if (errno == EAGAIN || (errno == EINTR && stopped)) continue;
            perror("VIDIOC_DQBUF"); break;
        }
        int64_t receipt = raw_ns(), offset;
        if (b.timestamp.tv_sec < 0 || b.timestamp.tv_sec > (INT64_MAX - 999999000LL) / 1000000000LL ||
            b.timestamp.tv_usec < 0 || b.timestamp.tv_usec >= 1000000 ||
            clock_offset(config.clock, hz, &offset)) {
            fprintf(stderr, "Invalid camera timestamp or clock mapping.\n"); break;
        }
        int64_t driver_stamp = (int64_t)b.timestamp.tv_sec * 1000000000LL + b.timestamp.tv_usec * 1000LL;
        if ((offset > 0 && driver_stamp > INT64_MAX - offset) ||
            (offset < 0 && driver_stamp < INT64_MIN - offset)) {
            fprintf(stderr, "Camera timestamp mapping overflow.\n"); break;
        }
        int64_t event = driver_stamp + offset;
        if (event <= last) { fprintf(stderr, "Nonmonotonic camera timestamp.\n"); break; }
        int64_t lag = receipt - event;
        if (lag < -5000000LL || lag > 250000000LL) {
            fprintf(stderr, "Unexpected camera clock/lag: event=%" PRId64 " lag_ms=%.2f. Recheck configured %s clock domain.\n",
                    event, (double)lag / 1e6, config.clock_name); break;
        }
        int64_t gray_start = timing_log ? raw_ns() : 0;
        if (b.index >= mapped || (b.flags & V4L2_BUF_FLAG_ERROR) || b.bytesused > p->sizeimage ||
            extract_gray(gray, layout.pixels, maps[b.index].data, b.bytesused, maps[b.index].length, &layout)) {
            fprintf(stderr, "Invalid camera frame at sequence %u.\n", b.sequence); break;
        }
        int64_t gray_done = timing_log ? raw_ns() : 0;
        struct v4l2_buffer q = {.type = type, .memory = V4L2_MEMORY_MMAP, .index = b.index};
        if (xioctl(fd, VIDIOC_QBUF, &q)) { perror("VIDIOC_QBUF"); break; }
        if (delivered) {
            uint32_t step = b.sequence - sequence;
            if (!step || step >= UINT32_C(0x80000000)) { fprintf(stderr, "Camera sequence repeated or moved backwards.\n"); break; }
            gaps += step - 1u;
        } else first = event;
        last = event;
        sequence = b.sequence;
        ++delivered;
        if (delivered == 1 || !(delivered % 150))
            fprintf(stderr, "GMSL frames=%" PRIu64 " seq_missing=%" PRIu64 " lag_ms=%.2f delivered_hz=%.1f\n",
                    delivered, gaps, (double)lag / 1e6, delivered > 1 ? (delivered - 1) * 1e9 / (event - first) : 0.0);
        int64_t write_start = timing_log ? raw_ns() : 0;
        if (fwrite(&event, sizeof(event), 1, stdout) != 1 ||
#ifdef ORB_CAMERA_SEQUENCE
            fwrite(&sequence, sizeof(sequence), 1, stdout) != 1 ||
#endif
            fwrite(gray, 1, layout.pixels, stdout) != layout.pixels || fflush(stdout)) {
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
cleanup:
    if (streaming && xioctl(fd, VIDIOC_STREAMOFF, &type)) perror("VIDIOC_STREAMOFF");
    for (unsigned i = 0; i < mapped; ++i) if (munmap(maps[i].data, maps[i].length)) perror("munmap camera buffer");
    if (fd >= 0) close(fd);
    free(gray);
    if (timing_log) fclose(timing_log);
    return 1; /* Parent termination/pipe closure ends a trial. */
}
