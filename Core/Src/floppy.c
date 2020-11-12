/*
 * floppy_generic.c
 *
 * Generic floppy drive low-level support routines.
 * Mainly dealing with IRQs, timers and DMA.
 *
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */
#include "main.h"
#include "util.h"
#include "timer.h"

#define m(bitnr) (1u<<(bitnr))

/* A DMA buffer for running a timer associated with a floppy-data I/O pin. */
struct dma_ring {
    /* Current state of DMA (RDATA):
     *  DMA_inactive: No activity, buffer is empty.
     *  DMA_starting: Buffer is filling, DMA+timer not yet active.
     *  DMA_active: DMA is active, timer is operational.
     *  DMA_stopping: DMA+timer halted, buffer waiting to be cleared.
     * Current state of DMA (WDATA):
     *  DMA_inactive: No activity, flux ring and bitcell buffer are empty.
     *  DMA_starting: Flux ring and bitcell buffer are filling.
     *  DMA_active: Writeback processing is active (to mass storage).
     *  DMA_stopping: Timer halted, buffers waiting to be cleared. */
#define DMA_inactive 0 /* -> {starting, active} */
#define DMA_starting 1 /* -> {active, stopping} */
#define DMA_active   2 /* -> {stopping} */
#define DMA_stopping 3 /* -> {inactive} */
    volatile uint8_t state;
    /* IRQ handler sets this if the read buffer runs dry. */
    volatile uint8_t kick_dma_irq;
    /* Indexes into the buf[] ring buffer. */
    uint16_t cons;
    union {
        uint16_t prod; /* dma_rd: our producer index for flux samples */
        uint16_t prev_sample; /* dma_wr: previous CCRx sample value */
    };
    /* DMA ring buffer of timer values (ARR or CCRx). */
    uint16_t buf[1024];
};

/* DMA buffers are permanently allocated while a disk image is loaded, allowing
 * independent and concurrent management of the RDATA/WDATA pins. */
static struct dma_ring *dma_rd; /* RDATA DMA buffer */
static struct dma_ring *dma_wr; /* WDATA DMA buffer */

/* Statically-allocated floppy drive state. Tracks head movements and
 * side changes at all times, even when the drive is empty. */
static struct drive {
    uint8_t cyl, head;
    bool_t writing;
    bool_t sel;
    bool_t index_suppressed; /* disable IDX while writing to USB stick */
    uint8_t outp;
    volatile bool_t inserted;
    struct timer chgrst_timer;
    struct {
        struct timer timer;
        bool_t on;
        bool_t changed;
    } motor;
    struct {
#define STEP_started  1 /* started by hi-pri IRQ */
#define STEP_latched  2 /* latched by lo-pri IRQ */
#define STEP_active   (STEP_started | STEP_latched)
#define STEP_settling 4 /* handled by step.timer */
        uint8_t state;
        bool_t inward;
        time_t start;
        struct timer timer;
    } step;
    uint32_t restart_pos;
    struct image *image;
} drive;

static struct image *image;

static struct {
    struct timer timer, timer_deassert;
    time_t prev_time;
    bool_t fake_fired;
} index;

static void rdata_stop(void);
static void wdata_start(void);
static void wdata_stop(void);

struct exti_irq {
    uint8_t irq, pri;
    uint16_t pr_mask; /* != 0: irq- and exti-pending flags are cleared */
};

#if defined(QUICKDISK)
#include "gotek/quickdisk.c"
#else
#include "gotek/floppy.c"
#endif

/* Initialise IRQs according to statically-defined exti_irqs[]. */
static void floppy_init_irqs(void)
{
    const struct exti_irq *e;
    unsigned int i;

    /* Configure physical interface interrupts. */
    for (i = 0, e = exti_irqs; i < ARRAY_SIZE(exti_irqs); i++, e++) {
        IRQx_set_prio(e->irq, e->pri);
        if (e->pr_mask != 0) {
            /* Do not trigger an initial interrupt on this line. Clear EXTI_PR
             * before IRQ-pending, otherwise IRQ-pending is immediately
             * reasserted. */
            exti->pr = e->pr_mask;
            IRQx_clear_pending(e->irq);
        } else {
            /* Common case: we deliberately trigger the first interrupt to
             * prime the ISR's state. */
            IRQx_set_pending(e->irq);
        }
    }

    /* Enable physical interface interrupts. */
    for (i = 0, e = exti_irqs; i < ARRAY_SIZE(exti_irqs); i++, e++) {
        IRQx_enable(e->irq);
    }
}

/* Allocate and initialise a DMA ring. */
static struct dma_ring *dma_ring_alloc(void)
{
    struct dma_ring *dma = arena_alloc(sizeof(*dma));
    memset(dma, 0, offsetof(struct dma_ring, buf));
    return dma;
}

/* Allocate floppy resources and mount the given image.
 * On return: dma_rd, dma_wr, image and index are all valid. */
static void floppy_mount(struct slot *slot)
{
    struct image *im;
    struct dma_ring *_dma_rd, *_dma_wr;
    struct drive *drv = &drive;
    FSIZE_t fastseek_sz;
    DWORD *cltbl;
    FRESULT fr;

    do {

        arena_init();

        _dma_rd = dma_ring_alloc();
        _dma_wr = dma_ring_alloc();

        im = arena_alloc(sizeof(*im));
        memset(im, 0, sizeof(*im));

        /* Create a fast-seek cluster table for the image. */
#define MAX_FILE_FRAGS 511 /* up to a 4kB cluster table */
        cltbl = arena_alloc(0);
        *cltbl = (MAX_FILE_FRAGS + 1) * 2;
        fatfs_from_slot(&im->fp, slot, FA_READ);
        fastseek_sz = f_size(&im->fp);
        if (fastseek_sz == 0) {
            /* Empty or dummy file. */
            cltbl = NULL;
        } else {
            im->fp.cltbl = cltbl;
            fr = f_lseek(&im->fp, CREATE_LINKMAP);
            printk("Fast Seek: %u frags\n", (*cltbl / 2) - 1);
            if (fr == FR_OK) {
                DWORD *_cltbl = arena_alloc(*cltbl * 4);
                ASSERT(_cltbl == cltbl);
            } else if (fr == FR_NOT_ENOUGH_CORE) {
                printk("Fast Seek: FAILED\n");
                cltbl = NULL;
            } else {
                F_die(fr);
            }
        }

        /* ~0 avoids sync match within fewer than 32 bits of scan start. */
        im->write_bc_window = ~0;

        /* Large buffer to absorb write latencies at mass-storage layer. */
        im->bufs.write_bc.len = 32*1024; /* 32kB, power of two. */
        im->bufs.write_bc.p = arena_alloc(im->bufs.write_bc.len);

        /* Read BC buffer overlaps the second half of the write BC buffer. This
         * is because:
         *  (a) The read BC buffer does not need to absorb such large latencies
         *      (reads are much more predictable than writes to mass storage).
         *  (b) By dedicating the first half of the write buffer to writes, we
         *      can safely start processing write flux while read-data is still
         *      processing (eg. in-flight mass storage io). At say 10kB of
         *      dedicated write buffer, this is good for >80ms before colliding
         *      with read buffers, even at HD data rate (1us/bitcell).
         *      This is more than enough time for read
         *      processing to complete. */
        im->bufs.read_bc.len = im->bufs.write_bc.len / 2;
        im->bufs.read_bc.p = (char *)im->bufs.write_bc.p
            + im->bufs.read_bc.len;

        /* Any remaining space is used for staging I/O to mass storage, shared
         * between read and write paths (Change of use of this memory space is
         * fully serialised). */
        im->bufs.write_data.len = arena_avail();
        im->bufs.write_data.p = arena_alloc(im->bufs.write_data.len);
        im->bufs.read_data = im->bufs.write_data;

        /* Minimum allowable buffer space. */
        ASSERT(im->bufs.read_data.len >= 10*1024);

        /* Mount the image file. */
        image_open(im, slot, cltbl);
        if (!im->disk_handler->write_track || volume_readonly())
            slot->attributes |= AM_RDO;
        if (slot->attributes & AM_RDO) {
            printk("Image is R/O\n");
        } else {
            image_extend(im);
        }

    } while (f_size(&im->fp) != fastseek_sz);

    /* After image is extended at mount time, we permit no further changes
     * to the file metadata. Clear the dirent info to ensure this. */
    im->fp.dir_ptr = NULL;
    im->fp.dir_sect = 0;

    _dma_rd->state = DMA_stopping;

    /* Make allocated state globally visible now. */
    drv->image = image = im;
    barrier(); /* image ptr /then/ dma rings */
    dma_rd = _dma_rd;
    dma_wr = _dma_wr;

    drv->index_suppressed = FALSE;
    index.prev_time = time_now();
}


