/*
 *  klog - facility to transfer buck kernel log between kernel and userspace
 *
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of Version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Copyright (C) IBM Corporation, 2005
 *
 * 2005		Tom Zanussi <zanussi@us.ibm.com>
 *
 */

#include <linux/module.h>
#include <linux/relay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

/* maximum size of klog formatting buffer beyond which truncation will occur */
#define KLOG_TMPBUF_SIZE (1024)
/* per-cpu klog formatting temporary buffer */
static char klog_buf[NR_CPUS][KLOG_TMPBUF_SIZE];

/* This app's channel/control files will appear in /debug/klog */
#define APP_DIR		"klog"

static struct rchan *chan = NULL;

/* app data */
static struct dentry *	dir;
static int		logging;
static int		mappings;
static int		suspended;
static size_t		dropped;
static size_t		subbuf_size = 262144;
static size_t		n_subbufs = 4;

/* channel-management control files */
static struct dentry	*enabled_control;
static struct dentry	*create_control;
static struct dentry	*subbuf_size_control;
static struct dentry	*n_subbufs_control;
static struct dentry	*dropped_control;

/* produced/consumed control files */
static struct dentry	*produced_control[NR_CPUS];
static struct dentry	*consumed_control[NR_CPUS];

/* control file fileop declarations */
struct file_operations	enabled_fops;
struct file_operations	create_fops;
struct file_operations	subbuf_size_fops;
struct file_operations	n_subbufs_fops;
struct file_operations	dropped_fops;
struct file_operations	produced_fops;
struct file_operations	consumed_fops;

/* forward declarations */
static int create_controls(void);
static void destroy_channel(void);
static void remove_controls(void);

/**
 *	module init - creates channel management control files
 *
 *	Returns 0 on success, negative otherwise.
 */
int vgt_klog_init(void)
{
	dir = debugfs_create_dir(APP_DIR, NULL);
	if (!dir) {
		printk("Couldn't create relay app directory.\n");
		return -ENOMEM;
	}

	if (create_controls()) {
		debugfs_remove(dir);
		return -ENOMEM;
	}

	return 0;
}

void vgt_klog_cleanup(void)
{
	destroy_channel();
	remove_controls();
	if (dir)
		debugfs_remove(dir);
}

/* Boilerplate code below here */

/**
 *	remove_channel_controls - removes produced/consumed control files
 */
static void remove_channel_controls(void)
{
	int i;

	for (i = 0; i < NR_CPUS; i++) {
		if (produced_control[i]) {
			debugfs_remove(produced_control[i]);
			produced_control[i] = NULL;
			continue;
		}
		break;
	}

	for (i = 0; i < NR_CPUS; i++) {
		if (consumed_control[i]) {
			debugfs_remove(consumed_control[i]);
			consumed_control[i] = NULL;
			continue;
		}
		break;
	}
}

/**
 *	create_channel_controls - creates produced/consumed control files
 *
 *	Returns channel on success, negative otherwise.
 */
static int create_channel_controls(struct dentry *parent,
				const char *base_filename,
				struct rchan *chan)
{
	unsigned int i;
	char *tmpname = kmalloc(NAME_MAX + 1, GFP_KERNEL);
	if (!tmpname)
		return -ENOMEM;

	for_each_online_cpu(i) {
		sprintf(tmpname, "%s%d.produced", base_filename, i);
		produced_control[i] = debugfs_create_file(tmpname, 0, parent, chan->buf[i], &produced_fops);
		if (!produced_control[i]) {
			printk("Couldn't create relay control file %s.\n",
				tmpname);
			goto cleanup_control_files;
		}

		sprintf(tmpname, "%s%d.consumed", base_filename, i);
		consumed_control[i] = debugfs_create_file(tmpname, 0, parent, chan->buf[i], &consumed_fops);
		if (!consumed_control[i]) {
			printk("Couldn't create relay control file %s.\n",
				tmpname);
			goto cleanup_control_files;
		}
	}
	kfree(tmpname);
	return 0;

cleanup_control_files:
	remove_channel_controls();
	kfree(tmpname);
	return -ENOMEM;
}

/*
 * subbuf_start() relay callback.
 *
 * Defined so that we can 1) reserve padding counts in the sub-buffers, and
 * 2) keep a count of events dropped due to the buffer-full condition.
 */
static int subbuf_start_handler(struct rchan_buf *buf,
				void *subbuf,
				void *prev_subbuf,
				size_t prev_padding)
{
	if (prev_subbuf)
		*((size_t *)prev_subbuf) = prev_padding;

	if (relay_buf_full(buf)) {
		if (!suspended) {
			suspended = 1;
			printk("cpu %d buffer full!!!\n", smp_processor_id());
		}
		dropped++;
		return 0;
	} else if (suspended) {
		suspended = 0;
		printk("cpu %d buffer no longer full.\n", smp_processor_id());
	}

	subbuf_start_reserve(buf, sizeof(size_t));

	return 1;
}

/*
 * file_create() callback.  Creates relay file in debugfs.
 */
static struct dentry *create_buf_file_handler(const char *filename,
						struct dentry *parent,
						umode_t mode,
						struct rchan_buf *buf,
						int *is_global)
{
	struct dentry *buf_file;

	buf_file = debugfs_create_file(filename, mode, parent, buf,
					&relay_file_operations);

	return buf_file;
}

/*
 * file_remove() default callback.  Removes relay file in debugfs.
 */
static int remove_buf_file_handler(struct dentry *dentry)
{
	debugfs_remove(dentry);

	return 0;
}

/*
 * relay callbacks
 */
static struct rchan_callbacks relay_callbacks =
{
	.subbuf_start = subbuf_start_handler,
	.create_buf_file = create_buf_file_handler,
	.remove_buf_file = remove_buf_file_handler,
};

/**
 *	create_channel - creates channel /debug/klog/cpuXXX
 *
 *	Creates channel along with associated produced/consumed control files
 *
 *	Returns channel on success, NULL otherwise
 */
static struct rchan *create_channel(unsigned subbuf_size,
					unsigned n_subbufs)
{
	struct rchan *chan;

	printk("create_channel: subbuf_size %u, n_subbufs %u, dir %p\n", subbuf_size, n_subbufs, dir);

	chan = relay_open("cpu", dir, subbuf_size,
			n_subbufs, &relay_callbacks, NULL);

	if (!chan) {
		printk("relay app channel creation failed\n");
		return NULL;
	}

	if (create_channel_controls(dir, "cpu", chan)) {
		relay_close(chan);
		return NULL;
	}

	logging = 0;
	mappings = 0;
	suspended = 0;
	dropped = 0;

	return chan;
}

/**
 *	destroy_channel - destroys channel /debug/APP_DIR/cpuXXX
 *
 *	Destroys channel along with associated produced/consumed control files
 */
static void destroy_channel(void)
{
	if (chan) {
		relay_close(chan);
		chan = NULL;
	}
	remove_channel_controls();
}

/**
 *	remove_controls - removes channel management control files
 */
static void remove_controls(void)
{
	if (enabled_control)
		debugfs_remove(enabled_control);

	if (subbuf_size_control)
		debugfs_remove(subbuf_size_control);

	if (n_subbufs_control)
		debugfs_remove(n_subbufs_control);

	if (create_control)
		debugfs_remove(create_control);

	if (dropped_control)
		debugfs_remove(dropped_control);
}

/**
 *	create_controls - creates channel management control files
 *
 *	Returns 0 on success, negative otherwise.
 */
static int create_controls(void)
{
	enabled_control = debugfs_create_file("enabled", 0, dir,
						NULL, &enabled_fops);
	if (!enabled_control) {
		printk("Couldn't create relay control file 'enabled'.\n");
		goto fail;
	}

	subbuf_size_control = debugfs_create_file("subbuf_size", 0, dir,
						NULL, &subbuf_size_fops);
	if (!subbuf_size_control) {
		printk("Couldn't create relay control file 'subbuf_size'.\n");
		goto fail;
	}

	n_subbufs_control = debugfs_create_file("n_subbufs", 0, dir,
						NULL, &n_subbufs_fops);
	if (!n_subbufs_control) {
		printk("Couldn't create relay control file 'n_subbufs'.\n");
		goto fail;
	}

	create_control = debugfs_create_file("create", 0, dir,
						NULL, &create_fops);
	if (!create_control) {
		printk("Couldn't create relay control file 'create'.\n");
		goto fail;
	}

	dropped_control = debugfs_create_file("dropped", 0, dir,
						NULL, &dropped_fops);
	if (!dropped_control) {
		printk("Couldn't create relay control file 'dropped'.\n");
		goto fail;
	}

	return 0;
fail:
	remove_controls();
	return -1;
}

/*
 * control file fileop definitions
 */

/*
 * control files for relay channel management
 */

static ssize_t enabled_read(struct file *filp, char __user *buffer,
				size_t count, loff_t *ppos)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%d\n", logging);
	return simple_read_from_buffer(buffer, count, ppos,
					buf, strlen(buf));
}

static ssize_t enabled_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	char buf[16];
	char *tmp;
	int enabled;

	if (count > sizeof(buf))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	enabled = simple_strtol(buf, &tmp, 10);
	if (tmp == buf)
		return -EINVAL;

	if (enabled && chan)
		logging = 1;
	else if (!enabled) {
		logging = 0;
		if (chan)
			relay_flush(chan);
	}

	return count;
}

/*
 * 'enabled' file operations - boolean r/w
 *
 *  toggles logging to the relay channel
 */
struct file_operations enabled_fops = {
	.owner	=	THIS_MODULE,
	.read	=	enabled_read,
	.write	=	enabled_write,
};

static ssize_t create_read(struct file *filp, char __user *buffer,
			size_t count, loff_t *ppos)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%d\n", !!chan);

	return simple_read_from_buffer(buffer, count, ppos,
					buf, strlen(buf));
}

static ssize_t create_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	char buf[16];
	char *tmp;
	int create;

	if (count > sizeof(buf))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	create = simple_strtol(buf, &tmp, 10);
	if (tmp == buf)
		return -EINVAL;

	if (create) {
		destroy_channel();
		chan = create_channel(subbuf_size, n_subbufs);
		if(!chan)
			return count;
	} else
		destroy_channel();

	return count;
}

/*
 * 'create' file operations - boolean r/w
 *
 *  creates/destroys the relay channel
 */
struct file_operations create_fops = {
	.owner	=	THIS_MODULE,
	.read	=	create_read,
	.write	=	create_write,
};

static ssize_t subbuf_size_read(struct file *filp, char __user *buffer,
				size_t count, loff_t *ppos)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%zu\n", subbuf_size);

	return simple_read_from_buffer(buffer, count, ppos,
					buf, strlen(buf));
}

static ssize_t subbuf_size_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	char buf[16];
	char *tmp;
	size_t size;

	if (count > sizeof(buf))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	size = simple_strtol(buf, &tmp, 10);
	if (tmp == buf)
		return -EINVAL;

	subbuf_size = size;

	return count;
}

/*
 * 'subbuf_size' file operations - r/w
 *
 *  gets/sets the subbuffer size to use in channel creation
 */
struct file_operations subbuf_size_fops = {
	.owner =	THIS_MODULE,
	.read =		subbuf_size_read,
	.write =	subbuf_size_write,
};

static ssize_t n_subbufs_read(struct file *filp, char __user *buffer,
				size_t count, loff_t *ppos)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%zu\n", n_subbufs);

	return simple_read_from_buffer(buffer, count, ppos,
					buf, strlen(buf));
}

static ssize_t n_subbufs_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	char buf[16];
	char *tmp;
	size_t n;

	if (count > sizeof(buf))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	n = simple_strtol(buf, &tmp, 10);
	if (tmp == buf)
		return -EINVAL;

	n_subbufs = n;

	return count;
}

/*
 * 'n_subbufs' file operations - r/w
 *
 *  gets/sets the number of subbuffers to use in channel creation
 */
struct file_operations n_subbufs_fops = {
	.owner =	THIS_MODULE,
	.read =		n_subbufs_read,
	.write =	n_subbufs_write,
};

static ssize_t dropped_read(struct file *filp, char __user *buffer,
				size_t count, loff_t *ppos)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%zu\n", dropped);

	return simple_read_from_buffer(buffer, count, ppos,
					buf, strlen(buf));
}

/*
 * 'dropped' file operations - r
 *
 *  gets the number of dropped events seen
 */
struct file_operations dropped_fops = {
	.owner =	THIS_MODULE,
	.read =		dropped_read,
};


/*
 * control files for relay produced/consumed sub-buffer counts
 */

static int produced_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;

	return 0;
}

static ssize_t produced_read(struct file *filp, char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct rchan_buf *buf = filp->private_data;

	return simple_read_from_buffer(buffer, count, ppos,
					&buf->subbufs_produced,
					sizeof(buf->subbufs_produced));
}

/*
 * 'produced' file operations - r, binary
 *
 *  There is a .produced file associated with each per-cpu relay file.
 *  Reading a .produced file returns the number of sub-buffers so far
 *  produced for the associated relay buffer.
 */
struct file_operations produced_fops = {
	.owner =	THIS_MODULE,
	.open =		produced_open,
	.read =		produced_read,
	.llseek = default_llseek,
};

static int consumed_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;

	return 0;
}

static ssize_t consumed_read(struct file *filp, char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct rchan_buf *buf = filp->private_data;

	return simple_read_from_buffer(buffer, count, ppos,
					&buf->subbufs_consumed,
					sizeof(buf->subbufs_consumed));
}

static ssize_t consumed_write(struct file *filp, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct rchan_buf *buf = filp->private_data;
	size_t consumed;

	if (copy_from_user(&consumed, buffer, sizeof(consumed)))
		return -EFAULT;

	relay_subbufs_consumed(buf->chan, buf->cpu, consumed);

	return count;
}

/**
 *	klog_printk - send a formatted string to the klog handler
 *	@fmt: format string, same as printk
 */

static int new_text_line[NR_CPUS] = {1};
void klog_printk(const char *fmt, ...)
{
	va_list args;
	int tlen, len,cpu,i;
	char *cbuf;
	char tbuf[KLOG_TMPBUF_SIZE];
	unsigned long flags;

	unsigned long long t;
	unsigned long nanosec_rem;

	if (!logging)
		return;

	local_irq_save(flags);

	cpu = smp_processor_id();
	cbuf = klog_buf[cpu];

	va_start(args, fmt);
	len = vsnprintf(tbuf, KLOG_TMPBUF_SIZE , fmt, args);
	va_end(args);

	for (i=0; i<len; i++){
		if (new_text_line[cpu]){
			/* Add the current time stamp */
			t = cpu_clock(cpu);
			nanosec_rem = do_div(t, 1000000000);
			tlen = sprintf(cbuf, "[%5lu.%06lu] ",
					(unsigned long) t,
					nanosec_rem / 1000);
			cbuf += tlen;
			new_text_line[cpu] = 0;
		}
		*cbuf++ = tbuf[i];
		if (tbuf[i] == '\n')
			new_text_line[cpu] = 1;
	}

	relay_write(chan, klog_buf[cpu], cbuf - klog_buf[cpu]);

	local_irq_restore(flags);
}

/*
 * 'consumed' file operations - r/w, binary
 *
 *  There is a .consumed file associated with each per-cpu relay file.
 *  Writing to a .consumed file adds the value written to the
 *  subbuffers-consumed count of the associated relay buffer.
 *  Reading a .consumed file returns the number of sub-buffers so far
 *  consumed for the associated relay buffer.
 */
struct file_operations consumed_fops = {
	.owner	=	THIS_MODULE,
	.open	=	consumed_open,
	.read	=	consumed_read,
	.write	=	consumed_write,
	.llseek	=	default_llseek,
};

