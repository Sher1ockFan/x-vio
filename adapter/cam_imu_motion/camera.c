/* Build on the Jetson: cc -O3 -std=c11 -Wall -Wextra -pthread camera.c -o camera */
#define _GNU_SOURCE
#include <stdint.h>
#include <stddef.h>
#include <arm_neon.h>
#define WIDTH 1920u
#define HEIGHT 1441u
#define STRIDE 3840u
#define FULL_SIZE (HEIGHT * STRIDE)
#define GRAY_SIZE (WIDTH * (HEIGHT - 1u))
#define META_SIZE 600u
#define WARMUP_US INT64_C(2000000)
#define IMAGE_STRIDE 2u

struct selection {
    int64_t first_ts_us;
    unsigned anchor_row;
    int anchored;
};

static int select_row(struct selection *s, unsigned row, int64_t ts_us) {
    if (row == 0) s->first_ts_us = ts_us;
    if (ts_us - s->first_ts_us < WARMUP_US) return 0;
    if (!s->anchored) { s->anchor_row = row; s->anchored = 1; }
    return (row - s->anchor_row) % IMAGE_STRIDE == 0;
}

/* Verified native format: row 0 is metadata; UYVY luma occupies byte 1 of each 2. */
static int extract_gray(uint8_t *gray, const uint8_t *src, size_t used) {
    if (used != FULL_SIZE) return -1;
    const uint8_t *pixels = src + STRIDE; /* Skip the metadata row exactly once. */
    const size_t vector_end = GRAY_SIZE - GRAY_SIZE % 16u;
    size_t i = 0;
    for (; i < vector_end; i += 16u) {
        /* UYVY: val[0] contains alternating U/V; val[1] contains 16 original Y bytes. */
        uint8x16x2_t pairs = vld2q_u8(pixels + 2u * i);
        vst1q_u8(gray + i, pairs.val[1]);
    }
    for (; i < GRAY_SIZE; ++i) gray[i] = pixels[2u * i + 1u];
    return 0;
}
#ifndef CAMERA_PROBE_PIXEL_ONLY
#if !defined(__linux__) || !defined(__aarch64__)
#error Compile on the Linux AArch64 Jetson.
#endif
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/statvfs.h>
#include <unistd.h>

#define DEVICE "/dev/video6"
#define EXPECT_CARD "vi-output, pnd_gmsl_mc 10-003c"
#define EXPECT_BUS "platform:tegra-capture-vi:2"
#define NS UINT64_C(1000000000)
#define WRITE_SLOTS 6u
#define DISK_RESERVE (UINT64_C(512) * 1024 * 1024)
static volatile sig_atomic_t stopped;
static void interrupted(int sig) { stopped = sig; }
static int control(int fd, unsigned long req, void *arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR && !stopped);
    return r;
}
static FILE *new_output(int dir, const char *name, const char *mode) {
    int fd = openat(dir, name, O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC, 0600);
    if (fd < 0) { perror(name); return NULL; }
    FILE *f = fdopen(fd, mode);
    if (!f) { perror("fdopen"); close(fd); }
    return f;
}
static int get_ns(clockid_t clk, uint64_t *ns) {
    struct timespec t;
    if (clock_gettime(clk, &t)) { perror("clock_gettime"); return -1; }
    *ns = (uint64_t)t.tv_sec * NS + (uint64_t)t.tv_nsec;
    return 0;
}
struct window { uint64_t begin, value, end; };
/* A virtual-counter read is bracketed by two CLOCK_MONOTONIC_RAW reads.
 * MONOTONIC and REALTIME get independent windows; these samples are not simultaneous. */
static int best_window(struct window *best, int kind, uint64_t frequency) {
    uint64_t shortest = UINT64_MAX;
    for (int i = 0; i < 20; ++i) {
        struct window w;
        if (get_ns(CLOCK_MONOTONIC_RAW, &w.begin)) return -1;
        if (kind == 0) {
            uint64_t count;
            __asm__ __volatile__("isb; mrs %0, cntvct_el0; isb" : "=r"(count) : : "memory");
            w.value = count / frequency * NS + (count % frequency) * NS / frequency;
        } else if (get_ns(kind == 1 ? CLOCK_MONOTONIC : CLOCK_REALTIME, &w.value)) return -1;
        if (get_ns(CLOCK_MONOTONIC_RAW, &w.end)) return -1;
        if (w.end >= w.begin && w.end - w.begin < shortest) {
            *best = w; shortest = w.end - w.begin;
        }
    }
    if (shortest == UINT64_MAX) { fprintf(stderr, "No valid RAW clock bracket.\n"); return -1; }
    return 0;
}
static int save_clocks(FILE *log, uint64_t frequency) {
    struct window w[3] = {{0}};
    for (int i = 0; i < 3; ++i) if (best_window(&w[i], i, frequency)) return -1;
    for (int i = 0; i < 3; ++i)
        if (fprintf(log, "%"PRIu64",%"PRIu64",%"PRIu64",", w[i].begin, w[i].value, w[i].end) < 0)
            return -1;
    return fprintf(log, "%"PRIu64"\n", frequency) < 0 || fflush(log) ? -1 : 0;
}
static int write_sample(int dir, const char *name, const uint8_t *gray) {
    FILE *f = new_output(dir, name, "wb");
    if (!f) return -1;
    int failed = fprintf(f, "P5\n%u %u\n255\n", WIDTH, HEIGHT-1u) < 0 ||
                 fwrite(gray, 1, GRAY_SIZE, f) != GRAY_SIZE;
    if (fclose(f)) failed = 1;
    if (failed) {
        fprintf(stderr, "Could not finish %s: %s. Keeping other completed files.\n", name, strerror(errno));
        if (unlinkat(dir,name,0)) perror("remove unfinished sample");
        return -1;
    }
    return 0;
}

/* Bound image memory and keep the DQBUF thread independent of filesystem writes. */
struct writer {
    int dir, stopping, failed;
    unsigned head, count, completed, ids[WRITE_SLOTS];
    uint8_t *pixels;
    pthread_mutex_t lock;
    pthread_cond_t ready;
    pthread_t thread;
};
static void *write_images(void *arg) {
    struct writer *w=arg;
    pthread_mutex_lock(&w->lock);
    for (;;) {
        while (!w->count && !w->stopping) pthread_cond_wait(&w->ready,&w->lock);
        if (!w->count) break;
        unsigned slot=w->head;
        char name[48];
        snprintf(name,sizeof(name),"sample_%06u.pgm",w->ids[slot]);
        pthread_mutex_unlock(&w->lock);
        int failed=write_sample(w->dir,name,w->pixels+(size_t)slot*GRAY_SIZE);
        pthread_mutex_lock(&w->lock);
        if (failed) { w->failed=1; break; }
        ++w->completed;
        w->head=(w->head+1) % WRITE_SLOTS;
        --w->count;
    }
    pthread_mutex_unlock(&w->lock);
    return NULL;
}

static int start_writer(struct writer *w, int dir) {
    w->dir=dir;
    w->pixels=malloc((size_t)WRITE_SLOTS*GRAY_SIZE);
    if (!w->pixels) { perror("allocate image queue"); return -1; }
    int error=pthread_create(&w->thread,NULL,write_images,w);
    if (error) { fprintf(stderr,"Start image writer: %s\n",strerror(error)); free(w->pixels); return -1; }
    return 0;
}
/* Return 1 if the bounded queue is full. Keep logging all frames in that case. */
static int queue_image(struct writer *w, unsigned row, const uint8_t *src, size_t used) {
    pthread_mutex_lock(&w->lock);
    int result=0;
    if (w->failed) result=-1;
    else if (w->count==WRITE_SLOTS) result=1;
    else {
        unsigned slot=(w->head+w->count) % WRITE_SLOTS;
        result=extract_gray(w->pixels+(size_t)slot*GRAY_SIZE,src,used);
        if (!result) {
            w->ids[slot]=row; ++w->count;
            pthread_cond_signal(&w->ready);
        }
    }
    pthread_mutex_unlock(&w->lock);
    return result;
}
static int finish_writer(struct writer *w) {
    pthread_mutex_lock(&w->lock);
    w->stopping=1;
    pthread_cond_signal(&w->ready);
    pthread_mutex_unlock(&w->lock);
    int error=pthread_join(w->thread,NULL);
    if (error) { fprintf(stderr,"Join image writer: %s\n",strerror(error)); return -1; }
    free(w->pixels);
    pthread_mutex_destroy(&w->lock);
    pthread_cond_destroy(&w->ready);
    return w->failed ? -1 : 0;
}

static int check_disk(int dir, uint64_t needed, int announce) {
    struct statvfs disk;
    if (fstatvfs(dir,&disk)) { perror("output filesystem space check"); return -1; }
    uint64_t unit=disk.f_frsize ? disk.f_frsize : disk.f_bsize;
    uint64_t free_blocks=disk.f_bavail;
    if (!unit || free_blocks < (needed+unit-1)/unit) {
        fprintf(stderr,"Storage guard: need %"PRIu64" available bytes, have %.0f. Retaining completed data.\n",
            needed, (double)free_blocks*(double)unit);
        return -1;
    }
    if (announce) fprintf(stderr,"DISK available_GiB=%.2f required_GiB=%.2f\n",
        (double)free_blocks*(double)unit/1073741824.0, (double)needed/1073741824.0);
    return 0;
}
struct mapping { void *ptr; size_t length; };
static int prepare_info(int dir, const struct v4l2_pix_format *p, unsigned duration,
                        uint64_t frequency, unsigned count) {
    FILE *f = new_output(dir, "capture_info.json", "w");
    if (!f) return -1;
    int n = fprintf(f, "{\n"
        "  \"schema_version\": 1,\n  \"device\": \"%s\",\n  \"driver\": \"tegra-video\",\n"
        "  \"card\": \"%s\",\n  \"bus\": \"%s\",\n  \"pixel_format\": \"UYVY\",\n"
        "  \"width\": %u,\n  \"height\": %u,\n  \"bytesperline\": %u,\n  \"sizeimage\": %u,\n"
        "  \"field\": %u,\n  \"active_first_row\": 1,\n  \"active_height\": 1440,\n"
        "  \"metadata_record_bytes\": %u,\n  \"requested_seconds\": %u,\n  \"mmap_buffer_count\": %u,\n"
        "  \"counter_frequency_hz\": %"PRIu64",\n"
        "  \"timestamp_note\": \"driver_ts_us is unchanged V4L2 timeval, flags are decimal; frame event and driver clock domain unverified\",\n"
        "  \"receipt_note\": \"RAW reads bracket nonblocking DQBUF after poll; MONOTONIC read follows DQBUF. No exposure or IMU correction.\",\n"
        "  \"warmup_driver_us\": 2000000,\n  \"image_stride\": 2,\n  \"expected_write_hz\": 15,\n"
        "  \"image_selection\": \"anchor at first delivered row with driver_ts_us - first delivered driver_ts_us >= warmup_driver_us; save anchor and alternating frame_index after anchor when valid\",\n"
        "  \"samples_note\": \"PGM preserves original UYVY luma. metadata_offset=-1 denotes rejected payload. sample names identify scheduled images; verify completeness against actual files.\"\n}\n",
        DEVICE, EXPECT_CARD, EXPECT_BUS, p->width,p->height,p->bytesperline,p->sizeimage,p->field,
        META_SIZE,duration,count,frequency);
    int failed = n < 0;
    if (fclose(f)) failed = 1;
    return failed ? -1 : 0;
}
int main(int argc, char **argv) {
    if (argc < 2 || argc > 3) { fprintf(stderr, "Usage: %s EXISTING_OUTDIR [SECONDS=65, 1..90]\n", argv[0]); return 2; }
    char *end = NULL;
    long duration = argc == 3 ? strtol(argv[2], &end, 10) : 65;
    if (duration < 1 || duration > 90 || (argc == 3 && (!*argv[2] || *end))) {
        fprintf(stderr, "SECONDS must be an integer from 1 to 90.\n"); return 2;
    }
    struct sigaction act = {0};
    act.sa_handler = interrupted; sigemptyset(&act.sa_mask);
    if (sigaction(SIGINT, &act, NULL) || sigaction(SIGTERM, &act, NULL)) { perror("sigaction"); return 1; }
    int rc = 1, dir = -1, fd = -1, streaming = 0;
    FILE *frames = NULL, *clocks = NULL, *meta = NULL;
    struct writer writer = {.lock=PTHREAD_MUTEX_INITIALIZER, .ready=PTHREAD_COND_INITIALIZER};
    int writer_started=0;
    struct selection image_policy={0};
    uint32_t previous_sequence=0;
    int64_t previous_driver_us=0;
    unsigned seq_anomalies=0, ts_anomalies=0, missed_images=0;
    struct mapping *maps = NULL;
    unsigned mapped = 0, good = 0, rows = 0, bad = 0, samples = 0;
    uint64_t frequency, t0, now, last_frame, last_clock, next_progress;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_capability cap = {0};
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    struct v4l2_requestbuffers req = { .count = 4, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                      .memory = V4L2_MEMORY_MMAP };
    dir = open(argv[1], O_RDONLY|O_DIRECTORY|O_CLOEXEC);
    if (dir < 0) { perror("output directory (must already exist)"); goto cleanup; }
    /* Budget for block allocation, sidecars, and a few extra startup frames. */
    unsigned image_budget=15u*(unsigned)(duration>2 ? duration-2 : 0)+4u;
    uint64_t space_needed=DISK_RESERVE+(uint64_t)image_budget*(GRAY_SIZE+4096u)+4u*1024u*1024u;
    if (check_disk(dir,space_needed,1)) goto cleanup;
    fd = open(DEVICE, O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if (fd < 0) { perror(DEVICE); goto cleanup; }
    if (control(fd, VIDIOC_QUERYCAP, &cap)) { perror("VIDIOC_QUERYCAP"); goto cleanup; }
    cap.card[sizeof(cap.card)-1] = cap.bus_info[sizeof(cap.bus_info)-1] = cap.driver[sizeof(cap.driver)-1] = 0;
    if (strcmp((char*)cap.driver,"tegra-video") || strcmp((char*)cap.card,EXPECT_CARD) ||
        strcmp((char*)cap.bus_info,EXPECT_BUS)) {
        fprintf(stderr, "Camera identity changed: driver=%s card=%s bus=%s. No capture.\n",
                (char*)cap.driver,(char*)cap.card,(char*)cap.bus_info); goto cleanup;
    }
    uint32_t caps = cap.capabilities & V4L2_CAP_DEVICE_CAPS ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_STREAMING) || !(caps & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr,"Camera lacks single-plane video capture or streaming.\n"); goto cleanup;
    }
    if (control(fd, VIDIOC_G_FMT, &fmt)) { perror("VIDIOC_G_FMT"); goto cleanup; }
    struct v4l2_pix_format *p = &fmt.fmt.pix;
    if (p->width != WIDTH || p->height != HEIGHT || p->pixelformat != V4L2_PIX_FMT_UYVY ||
        p->bytesperline != STRIDE || p->sizeimage != FULL_SIZE || p->field != V4L2_FIELD_NONE) {
        fprintf(stderr,"Unexpected format: %ux%u fourcc=0x%08x stride=%u size=%u field=%u. No changes made.\n",
                p->width,p->height,p->pixelformat,p->bytesperline,p->sizeimage,p->field); goto cleanup;
    }
    __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(frequency));
    if (!frequency || frequency > UINT32_MAX) { fprintf(stderr,"Unexpected CNTFRQ.\n"); goto cleanup; }
    if (start_writer(&writer,dir)) goto cleanup;
    writer_started=1;
    if (control(fd, VIDIOC_REQBUFS, &req)) { perror("VIDIOC_REQBUFS (close camera preview first)"); goto cleanup; }
    if (!req.count || req.count > 64) { fprintf(stderr, "Unexpected MMAP buffer count %u\n", req.count); goto cleanup; }
    maps = calloc(req.count,sizeof(*maps));
    if (!maps) { perror("calloc mappings"); goto cleanup; }
    for (unsigned i = 0; i < req.count; ++i) {
        struct v4l2_buffer b = {.type=type,.memory=V4L2_MEMORY_MMAP,.index=i};
        if (control(fd, VIDIOC_QUERYBUF, &b)) { perror("VIDIOC_QUERYBUF"); goto cleanup; }
        if (b.length < FULL_SIZE) { fprintf(stderr,"Buffer %u too short (%u bytes).\n", i,b.length); goto cleanup; }
        maps[i].length = b.length;
        maps[i].ptr = mmap(NULL,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
        if (maps[i].ptr == MAP_FAILED) { perror("mmap"); goto cleanup; }
        ++mapped;
        if (control(fd, VIDIOC_QBUF, &b)) { perror("VIDIOC_QBUF"); goto cleanup; }
    }
    if (prepare_info(dir,p,(unsigned)duration,frequency,req.count)) goto cleanup;
    frames = new_output(dir,"camera_frames.csv","w");
    clocks = new_output(dir,"clock_samples.csv","w");
    meta = new_output(dir,"camera_metadata.bin","wb");
    if (!frames || !clocks || !meta) goto cleanup;
    if (fputs("frame_index,sequence,index,flags,driver_ts_us,raw_before_ns,raw_after_ns,mono_after_ns,bytesused,metadata_offset,sample\n",frames)==EOF ||
        fputs("raw_before_ns,counter_ns,raw_after_ns,mono_raw_before_ns,mono_ns,mono_raw_after_ns,realtime_raw_before_ns,realtime_ns,realtime_raw_after_ns,cntfrq_hz\n",clocks)==EOF) {
        perror("write headers"); goto cleanup;
    }
    if (save_clocks(clocks,frequency)) { perror("first clock sample"); goto cleanup; }
    if (stopped) goto cleanup;
    if (control(fd, VIDIOC_STREAMON, &type)) { perror("VIDIOC_STREAMON"); goto cleanup; }
    streaming = 1;
    if (get_ns(CLOCK_MONOTONIC,&t0)) goto cleanup;
    last_frame=last_clock=t0; next_progress=t0+5u*NS;
    fprintf(stderr,"CAMERA_STARTED device=%s duration_s=%ld\n",DEVICE,duration);
    while (!stopped) {
        if (get_ns(CLOCK_MONOTONIC,&now)) goto cleanup;
        if (now - last_frame >= 3u*NS) { fprintf(stderr,"No dequeued frame for 3 s.\n"); goto cleanup; }
        if (now - t0 >= (uint64_t)duration * NS) break;
        if (now - last_clock >= NS) {
            if (save_clocks(clocks,frequency) || fflush(frames) || fflush(meta)) { perror("flush logs"); goto cleanup; }
            /* Leave room for the time until the next check and all queued images. */
            if (check_disk(dir,DISK_RESERVE+(uint64_t)(WRITE_SLOTS+16u)*(GRAY_SIZE+4096u),0)) goto cleanup;
            last_clock=now;
        }
        if (now >= next_progress) {
            fprintf(stderr,"CAMERA_PROGRESS elapsed_s=%.1f rows=%u accepted=%u images_queued=%u storage_skips=%u\n",
                    (double)(now-t0)/(double)NS,rows,good,samples,missed_images);
            fflush(stderr);
            next_progress=now+5u*NS;
        }
        struct pollfd wait = {.fd=fd,.events=POLLIN};
        int ready = poll(&wait,1,100);
        if (ready < 0 && errno==EINTR) continue;
        if (ready < 0) { perror("poll"); goto cleanup; }
        if (!ready) continue;
        if (!(wait.revents & POLLIN) || (wait.revents & (POLLNVAL|POLLHUP))) {
            fprintf(stderr,"Camera poll error revents=0x%x\n", (unsigned)wait.revents); goto cleanup;
        }
        struct v4l2_buffer b = {.type=type,.memory=V4L2_MEMORY_MMAP};
        uint64_t raw0,raw1,mono;
        if (get_ns(CLOCK_MONOTONIC_RAW,&raw0)) goto cleanup;
        int dq = control(fd,VIDIOC_DQBUF,&b);
        int dq_errno = errno;
        if (get_ns(CLOCK_MONOTONIC_RAW,&raw1) || get_ns(CLOCK_MONOTONIC,&mono)) goto cleanup;
        if (dq < 0 && (dq_errno==EINTR || dq_errno==EAGAIN)) continue;
        if (dq < 0) { errno=dq_errno; perror("VIDIOC_DQBUF"); goto cleanup; }
        last_frame=mono;
        if (b.index >= mapped) { fprintf(stderr,"Out-of-range DQBUF index %u.\n",b.index); goto cleanup; }
        const uint8_t *src = maps[b.index].ptr;
        int valid = b.bytesused == FULL_SIZE && !(b.flags & V4L2_BUF_FLAG_ERROR);
        int64_t driver_us = (int64_t)b.timestamp.tv_sec * INT64_C(1000000) + (int64_t)b.timestamp.tv_usec;
        int selected=select_row(&image_policy,rows,driver_us);
        if (rows && b.sequence!=(uint32_t)(previous_sequence+1u)) {
            ++seq_anomalies;
            fprintf(stderr,"CAMERA_SEQ_ANOMALY row=%u previous=%u current=%u\n",rows,previous_sequence,b.sequence);
        }
        int64_t interval=driver_us-previous_driver_us;
        if (driver_us<=0 || (rows && (interval<20000 || interval>50000))) {
            ++ts_anomalies;
            fprintf(stderr,"CAMERA_TS_ANOMALY row=%u ts_us=%"PRId64" delta_us=%"PRId64"\n",rows,driver_us,interval);
        }
        previous_sequence=b.sequence; previous_driver_us=driver_us;
        int take_sample = valid && selected;
        int writer_error = 0;
        uint8_t head[META_SIZE];
        char name[48] = "";
        if (valid) memcpy(head,src,META_SIZE);
        if (take_sample) {
            int queued=queue_image(&writer,rows,src,b.bytesused);
            if (queued < 0) {
                writer_error = 1;
            } else if (queued > 0) {
                ++missed_images;
                fprintf(stderr,"CAMERA_IMAGE_SKIP row=%u reason=writer_queue_full\n",rows);
            } else {
                snprintf(name,sizeof(name),"sample_%06u.pgm",rows);
                ++samples;
            }
        }
        if (!valid) {
            ++bad;
            fprintf(stderr,"Rejected frame %u seq=%u flags=0x%x bytesused=%u\n",rows,b.sequence,b.flags,b.bytesused);
        }
        /* QBUF uses a fresh descriptor: it must not modify the original DQBUF log fields. */
        struct v4l2_buffer recycle = {.type=type,.memory=V4L2_MEMORY_MMAP,.index=b.index};
        int qr = control(fd,VIDIOC_QBUF,&recycle), q_errno = errno;
        if (valid && fwrite(head,1,META_SIZE,meta)!=META_SIZE) { perror("metadata write"); goto cleanup; }
        int64_t offset = valid ? (int64_t)good*META_SIZE : -1;
        if (fprintf(frames,"%u,%u,%u,%u,%"PRId64",%"PRIu64",%"PRIu64",%"PRIu64",%u,%"PRId64",%s\n",
              rows,b.sequence,b.index,b.flags,driver_us,raw0,raw1,mono,b.bytesused,offset,name) < 0) {
            perror("frame log write"); goto cleanup;
        }
        if (valid) ++good;
        ++rows;
        if (qr) { errno=q_errno; perror("VIDIOC_QBUF after dequeue"); goto cleanup; }
        if (writer_error) { fprintf(stderr,"Image writer failed. Keeping available records.\n"); goto cleanup; }
    }
    if (save_clocks(clocks,frequency)) { perror("last clock sample"); goto cleanup; }
    if (stopped) fprintf(stderr,"Capture interrupted by signal %d.\n",(int)stopped);
    if (!good) fprintf(stderr,"No valid frames captured.\n");
    /* A completed capture with rejected frames is still useful to the joint analyzer. */
    rc = stopped || !good ? 1 : (bad || missed_images ? 2 : 0);
cleanup:
    if (streaming && control(fd, VIDIOC_STREAMOFF, &type)) { perror("VIDIOC_STREAMOFF"); rc=1; }
    if (writer_started && finish_writer(&writer)) rc=1;
    for (unsigned i=0; i<mapped; ++i) if (munmap(maps[i].ptr,maps[i].length)) { perror("munmap"); rc=1; }
    free(maps);
    if (fd >= 0 && close(fd)) { perror("close camera"); rc=1; }
    if (frames && fclose(frames)) { perror("camera_frames.csv close"); rc=1; }
    if (clocks && fclose(clocks)) { perror("clock_samples.csv close"); rc=1; }
    if (meta && fclose(meta)) { perror("camera_metadata.bin close"); rc=1; }
    if (dir >= 0) close(dir);
    fprintf(stderr,"CAMERA_FINISHED frames=%u accepted=%u rejected=%u samples=%u seq_anomalies=%u ts_anomalies=%u storage_skips=%u status=%d\n",
            rows,good,bad,writer.completed,seq_anomalies,ts_anomalies,missed_images,rc);
    return rc;
}
#endif

