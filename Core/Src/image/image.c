/*
 * image.c
 * 
 * Interface for accessing image files.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#include "util.h"
#include "stdint.h"
#include "floppy.h"
#include "string.h"
//#include "ff.h"

extern const struct image_handler adf_image_handler;
extern const struct image_handler dummy_image_handler;

const struct image_type image_type[] = {
    { "adf", &adf_image_handler },
    { "", NULL }
};

uint8_t image_valid(FILINFO *fp)
{
    char ext[8];

    /* Skip directories. */
    if (fp->fattrib & AM_DIR)
        return FALSE;

    /* Skip empty images. */
    if (fp->fsize == 0)
        return FALSE;

    /* Check valid extension. */
    filename_extension(fp->fname, ext, sizeof(ext));
    if (!strcmp(ext, "adf")) {
        return  !(fp->fsize % (2*11*512));
    } else {
        const struct image_type *type;
        for (type = &image_type[0]; type->handler != NULL; type++)
            if (!strcmp(ext, type->ext))
                return TRUE;
    }

    return FALSE;
}

static uint8_t try_handler(struct image *im, struct slot *slot,
                          DWORD *cltbl,
                          const struct image_handler *handler)
{
    struct image_bufs bufs = im->bufs;
    BYTE mode;

    /* Reinitialise image structure, except for static buffers. */
    memset(im, 0, sizeof(*im));
    im->bufs = bufs;
    im->cur_track = ~0;
    im->slot = slot;

    /* Sensible defaults. */
    im->sync = SYNC_mfm;
    im->write_bc_ticks = sysclk_us(2);
    im->stk_per_rev = stk_ms(200);

    im->disk_handler = im->track_handler = handler;

    mode = FA_READ | FA_OPEN_EXISTING;
    if (handler->write_track != NULL)
        mode |= FA_WRITE;
    fatfs_from_slot(&im->fp, slot, mode);
    im->fp.cltbl = cltbl;

    return handler->open(im);
}


void image_open(struct image *im, struct slot *slot, DWORD *cltbl)
{
    static const struct image_handler * const image_handlers[] = {
        /* Special handler for dummy slots (empty HxC slot 0). */
        &dummy_image_handler,
        /* Only put formats here that have a strong identifying header. */
    };

    char ext[sizeof(slot->type)+1];
    const struct image_handler *hint;
    const struct image_type *type;
    int i;

    /* Extract filename extension (if available). */
    memcpy(ext, slot->type, sizeof(slot->type));
    ext[sizeof(slot->type)] = '\0';

    /* Use the extension as a hint to the correct image handler. */
    for (type = &image_type[0]; type->handler != NULL; type++)
        if (!strcmp(ext, type->ext))
            break;
    hint = type->handler;



    /* Filename extension hinting failed: walk the handler list. */
    for (i = 0; i < ARRAY_SIZE(image_handlers); i++) {
        if (try_handler(im, slot, cltbl, image_handlers[i]))
            return;
    }

    /* No handler found: bad image. */
    F_die(FR_BAD_IMAGE);
}

void image_extend(struct image *im)
{
    FSIZE_t new_sz;
    return;
/*
    if (!(im->disk_handler->extend && im->fp.dir_ptr && ff_cfg.extend_image))
        return;

    new_sz = im->disk_handler->extend(im);
    if (f_size(&im->fp) >= new_sz)
        return;

    /* Disable fast-seek mode, as it disallows extending the file. */
   // im->fp.cltbl = NULL;

    /* Attempt to extend the file. */
   // F_lseek(&im->fp, new_sz);
   // F_sync(&im->fp);
   // if (f_tell(&im->fp) != new_sz)
   //     F_die(FR_DISK_FULL);

    /* Update the slot for the new file size. */
    //im->slot->size = new_sz;*/
}

static void print_image_info(struct image *im)
{
    char msg[25];
    const static char *sync_s[] = { "Raw", "FM", "MFM" };
    const static char dens_c[] = { 'S', 'D', 'H', 'E' };
    int tlen, i;

    i = 0;
    for (tlen = 75000; tlen < im->tracklen_bc; tlen *= 2) {
        if (i == (sizeof(dens_c)-1))
            break;
        i++;
    }
    snprintf(msg, sizeof(msg), "%s %cS/%cD %uT",
             sync_s[im->sync],
             (im->nr_sides == 1) ? 'S' : 'D',
             dens_c[i], im->nr_cyls);
    lcd_write(0, 2, -1, msg);
}

uint8_t image_setup_track(
    struct image *im, uint16_t track, uint32_t *start_pos)
{
    const struct image_handler *h = im->track_handler;


    im->track_handler = h;
    h->setup_track(im, track, start_pos);

    print_image_info(im);

    return FALSE;
}

uint8_t image_read_track(struct image *im)
{
    return im->track_handler->read_track(im);
}

uint16_t image_rdata_flux(struct image *im, uint16_t *tbuf, uint16_t nr)
{
    return im->track_handler->rdata_flux(im, tbuf, nr);
}

uint8_t image_write_track(struct image *im)
{
    return im->track_handler->write_track(im);
}

uint32_t image_ticks_since_index(struct image *im)
{
    uint32_t ticks = im->cur_ticks - im->ticks_since_flux;
    if ((int32_t)ticks < 0)
        ticks += im->tracklen_ticks;

    return ticks;
}


