/*
 * util.h
 * 
 * Utility definitions.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */
#include "stdint.h"
#include "stdio.h"



#define ASSERT(p) do { if (0 && (p)) {} } while (0)

#define FF_USE_LFN		1
#define FF_MAX_LFN		255

typedef char bool_t;
#define TRUE 1
#define FALSE 0

#ifndef offsetof
#define offsetof(a,b) __builtin_offsetof(a,b)
#endif
#define container_of(ptr, type, member) ({                      \
        typeof( ((type *)0)->member ) *__mptr = (ptr);          \
        (type *)( (char *)__mptr - offsetof(type,member) );})
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define min(x,y) ({                             \
    const typeof(x) _x = (x);                   \
    const typeof(y) _y = (y);                   \
    (void) (&_x == &_y);                        \
    _x < _y ? _x : _y; })

#define max(x,y) ({                             \
    const typeof(x) _x = (x);                   \
    const typeof(y) _y = (y);                   \
    (void) (&_x == &_y);                        \
    _x > _y ? _x : _y; })

#define min_t(type,x,y) \
    ({ type __x = (x); type __y = (y); __x < __y ? __x: __y; })
#define max_t(type,x,y) \
    ({ type __x = (x); type __y = (y); __x > __y ? __x: __y; })
#define range_t(type,x,a,b) min_t(type, max_t(type, x, a), b)

struct slot {
    char name[FF_MAX_LFN+1];
    char type[7];
    uint8_t attributes;
    uint32_t firstCluster;
    uint32_t size;
    uint32_t dir_sect, dir_ptr;
};


/*
void fatfs_from_slot(FIL *file, const struct slot *slot, BYTE mode);

void filename_extension(const char *filename, char *extension, size_t size);
*/
/* Fast memset/memcpy: Pointers must be word-aligned, count must be a non-zero 
 * multiple of 32 bytes. */

void memset_fast(void *s, int c, size_t n);
void memcpy_fast(void *dest, const void *src, size_t n);

void *memset(void *s, int c, size_t n);
void *memcpy(void *dest, const void *src, size_t n);
void *memmove(void *dest, const void *src, size_t n);
int memcmp(const void *s1, const void *s2, size_t n);



long int strtol(const char *nptr, char **endptr, int base);

void qsort_p(void *base, unsigned int nr,
             int (*compar)(const void *, const void *));

//uint32_t rand(void);

unsigned int popcount(uint32_t x);

int vsnprintf(char *str, size_t size, const char *format, va_list ap)
    __attribute__ ((format (printf, 3, 0)));

int snprintf(char *str, size_t size, const char *format, ...)
    __attribute__ ((format (printf, 3, 4)));


#define le16toh(x) (x)
#define le32toh(x) (x)
#define htole16(x) (x)
#define htole32(x) (x)
#define be16toh(x) _rev16(x)
//#define be32toh(x) _rev32(x)
#define htobe16(x) _rev16(x)
//#define htobe32(x) _rev32(x)

int tolower(int c);

/* USB stack processing
void usbh_msc_init(void);
void usbh_msc_buffer_set(uint8_t *buf);
void usbh_msc_process(void);
bool_t usbh_msc_inserted(void);
*/
