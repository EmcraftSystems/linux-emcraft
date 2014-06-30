#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/io.h>

/*
 * Copy data from IO memory space to "real" memory space.
 * This needs to be optimized.
 */
void _memcpy_fromio(void *to, const void __iomem *from, size_t count)
{
	unsigned char *t = to;
	while (count) {
		count--;
		*t = readb(from);
		t++;
		from++;
	}
}

void _memcpy_fromiow(void *to, const void __iomem *from, size_t count)
{
	unsigned short *t = to;
	const unsigned short __iomem *f = from;

	BUG_ON(count % 2);
	while (count) {
		count -= 2;
		put_unaligned(get_unaligned(f), t);
		t++;
		f++;
	}
}

void _memcpy_fromiol(void *to, const void __iomem *from, size_t count)
{
	unsigned long *t = to;
	const unsigned long __iomem *f = from;

	BUG_ON(count % 4);
	while (count) {
		count -= 4;
		put_unaligned(get_unaligned(f), t);
		t++;
		f++;
	}
}

/*
 * Copy data from "real" memory space to IO memory space.
 * This needs to be optimized.
 */
void _memcpy_toio(void __iomem *to, const void *from, size_t count)
{
	const unsigned char *f = from;
	while (count) {
		count--;
		writeb(*f, to);
		f++;
		to++;
	}
}

void _memcpy_toiow(void __iomem *to, const void *from, size_t count)
{
	const unsigned short *f = from;

	BUG_ON(count % 2);
	while (count) {
		count -= 2;
		writew(get_unaligned(f), to);
		f++;
		to += 2;
	}
}

void _memcpy_toiol(void __iomem *to, const void *from, size_t count)
{
	const unsigned long *f = from;

	BUG_ON(count % 4);
	while (count) {
		count -= 4;
		writel(get_unaligned(f), to);
		f++;
		to += 4;
	}
}

/*
 * "memset" on IO memory space.
 * This needs to be optimized.
 */
void _memset_io(void __iomem *dst, int c, size_t count)
{
	while (count) {
		count--;
		writeb(c, dst);
		dst++;
	}
}

EXPORT_SYMBOL(_memcpy_fromio);
EXPORT_SYMBOL(_memcpy_fromiow);
EXPORT_SYMBOL(_memcpy_fromiol);
EXPORT_SYMBOL(_memcpy_toio);
EXPORT_SYMBOL(_memcpy_toiow);
EXPORT_SYMBOL(_memcpy_toiol);
EXPORT_SYMBOL(_memset_io);
