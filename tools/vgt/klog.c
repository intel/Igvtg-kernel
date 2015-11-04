/*
 * klog - log klog trace data
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (C) 2005 - Tom Zanussi (zanussi@us.ibm.com), IBM Corp
 *
 * Usage:
 *
 * mount -t debugfs debugfs /debug
 * insmod ./klog-mod.ko
 * ./klog [-b subbuf-size -n n_subbufs]
 *
 * captured output will appear in ./cpu0...cpuN-1
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <pthread.h>

/* name of directory containing relay files */
char *app_dirname = "/sys/kernel/debug/klog";
/* base name of per-cpu relay files (e.g. /debug/klog/cpu0, cpu1, ...) */
char *percpu_basename = "cpu";
/* base name of per-cpu output files (e.g. ./cpu0, cpu1, ...) */
char *percpu_out_basename = "cpu";

/* maximum number of CPUs we can handle - change if more */
#define NR_CPUS 256

/* internal variables */
static size_t subbuf_size = 524288; /* 512K */
static size_t n_subbufs = 8;
static unsigned int ncpus;
static int processing;
static pthread_mutex_t processing_mutex;

/* per-cpu internal variables */
static int relay_file[NR_CPUS];
static int out_file[NR_CPUS];
static char *relay_buffer[NR_CPUS];
static pthread_t reader[NR_CPUS];

/* control files */
static int produced_file[NR_CPUS];
static int consumed_file[NR_CPUS];

/* per-cpu buffer info */
static struct buf_status
{
	size_t produced;
	size_t consumed;
	size_t max_backlog; /* max # sub-buffers ready at one time */
} status[NR_CPUS];

static void usage(void)
{
	fprintf(stderr, "klog [-b subbuf_size -n n_subbufs]\n");
	exit(1);
}

/* Boilerplate code below here */

/**
 *	process_subbufs - write ready subbufs to disk
 */
static int process_subbufs(unsigned int cpu)
{
	size_t i, start_subbuf, end_subbuf, subbuf_idx, subbufs_consumed = 0;
	size_t subbufs_ready = status[cpu].produced - status[cpu].consumed;
	char *subbuf_ptr;
	size_t padding;
	int len;

	start_subbuf = status[cpu].consumed % n_subbufs;
	end_subbuf = start_subbuf + subbufs_ready;
	for (i = start_subbuf; i < end_subbuf; i++) {
		subbuf_idx = i % n_subbufs;
		subbuf_ptr = relay_buffer[cpu] + subbuf_idx * subbuf_size;
		padding = *((size_t *)subbuf_ptr);
		subbuf_ptr += sizeof(padding);
		len = (subbuf_size - sizeof(padding)) - padding;
		if (write(out_file[cpu], subbuf_ptr, len) < 0) {
			printf("Couldn't write to output file for cpu %d, exiting: errcode = %d: %s\n", cpu, errno, strerror(errno));
			exit(1);
		}
		subbufs_consumed++;
	}

	return subbufs_consumed;
}

/**
 *	check_buffer - check for and read any available sub-buffers in a buffer
 */
static void check_buffer(unsigned cpu)
{
	size_t subbufs_consumed;

	lseek(produced_file[cpu], 0, SEEK_SET);
	if (read(produced_file[cpu], &status[cpu].produced,
		 sizeof(status[cpu].produced)) < 0) {
		printf("Couldn't read from consumed file for cpu %d, exiting: errcode = %d: %s\n", cpu, errno, strerror(errno));
		exit(1);
	}

	subbufs_consumed = process_subbufs(cpu);
	if (subbufs_consumed) {
		if (subbufs_consumed == n_subbufs)
			fprintf(stderr, "cpu %d buffer full.  Consider using a larger buffer size.\n", cpu);
		if (subbufs_consumed > status[cpu].max_backlog)
			status[cpu].max_backlog = subbufs_consumed;
		status[cpu].consumed += subbufs_consumed;
		if (write(consumed_file[cpu], &subbufs_consumed,
			  sizeof(subbufs_consumed)) < 0) {
			printf("Couldn't write to consumed file for cpu %d, exiting: errcode = %d: %s\n", cpu, errno, strerror(errno));
			exit(1);
		}
	}
}

/**
 *	reader_thread - per-cpu channel buffer reader
 */
static void *reader_thread(void *data)
{
	int rc;
	unsigned long cpu = (unsigned long)data;
	struct pollfd pollfd;

	do {
		pollfd.fd = relay_file[cpu];
		pollfd.events = POLLIN;
		rc = poll(&pollfd, 1, -1);
		if (rc < 0) {
			if (errno != EINTR) {
				printf("poll error: %s\n",strerror(errno));
				exit(1);
			}
			printf("poll warning: %s\n",strerror(errno));
			rc = 0;
		}
		pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
		pthread_mutex_lock(&processing_mutex);
		processing++;
		pthread_mutex_unlock(&processing_mutex);
		check_buffer(cpu);
		pthread_mutex_lock(&processing_mutex);
		processing--;
		pthread_mutex_unlock(&processing_mutex);
		pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	} while (1);
}

/**
 *	control_read - read a control file and return the value read
 */
static size_t control_read(const char *dirname,
			   const char *filename)
{
	char tmp[4096];
	int fd;

	sprintf(tmp, "%s/%s", dirname, filename);
	fd = open(tmp, O_RDONLY);
	if (fd < 0) {
		printf("Couldn't open control file %s\n", tmp);
		exit(1);
	}

	if (read(fd, tmp, sizeof(tmp)) < 0) {
		printf("Couldn't read control file %s: errcode = %d: %s\n",
		       tmp, errno, strerror(errno));
		close(fd);
		exit(1);
	}

	close(fd);

	return atoi(tmp);
}

/**
 *	control_read - write a value to a control file
 */
static void control_write(const char *dirname,
			  const char *filename,
			  size_t val)
{
	char tmp[4096];
	int fd;

	sprintf(tmp, "%s/%s", dirname, filename);
	fd = open(tmp, O_RDWR);
	if (fd < 0) {
		printf("Couldn't open control file %s\n", tmp);
		exit(1);
	}

	sprintf(tmp, "%zu", val);

	if (write(fd, tmp, strlen(tmp)) < 0) {
		printf("Couldn't write control file %s: errcode = %d: %s\n",
		       tmp, errno, strerror(errno));
		close(fd);
		exit(1);
	}

	close(fd);
}

static void summarize(void)
{
	int i;
	size_t dropped;

	printf("summary:\n");
	for (i = 0; i < ncpus; i++) {
		printf("  cpu %u:\n", i);
		printf("    %zu sub-buffers processed\n",
		       status[i].consumed);
		printf("    %zu max backlog\n", status[i].max_backlog);
		printf("    data stored in file ./cpu%d\n", i);
	}

	dropped = control_read(app_dirname, "dropped");
	if (dropped)
		printf("\n    %zu dropped events.\n", dropped);
}

/**
 *      create_percpu_threads - create per-cpu threads
 */
static int create_percpu_threads(void)
{
	unsigned long i;

	for (i = 0; i < ncpus; i++) {
		/* create a thread for each per-cpu buffer */
		if (pthread_create(&reader[i], NULL, reader_thread,
				   (void *)i) < 0) {
			printf("Couldn't create thread\n");
			control_write(app_dirname, "enabled", 0);
			control_write(app_dirname, "create", 0);
			return -1;
		}
	}

	return 0;
}

/**
 *      kill_percpu_threads - kill per-cpu threads 0->n-1
 *      @n: number of threads to kill
 *
 *      Returns number of threads killed.
 */
static int kill_percpu_threads(int n)
{
        int i, killed = 0, err;

        for (i = 0; i < n; i++) {
                if ((err = pthread_cancel(reader[i])) == 0)
			killed++;
		else
			fprintf(stderr, "WARNING: couldn't kill per-cpu thread %d, err = %d\n", i, err);
        }

        if (killed != n)
                fprintf(stderr, "WARNING: couldn't kill all per-cpu threads:  %d killed, %d total\n", killed, n);

        return killed;
}

/**
 *	close_control_files - open per-cpu produced/consumed control files
 */
static void close_control_files(void)
{
	int i;

	for (i = 0; i < ncpus; i++) {
		if (produced_file[i] > 0)
			close(produced_file[i]);
		if (consumed_file[i] > 0)
			close(consumed_file[i]);
	}
}

/**
 *	open_control_files - open per-cpu produced/consumed control files
 */
static int open_control_files(const char *dirname, const char *basename)
{
	int i;
	char tmp[4096];

	for (i = 0; i < ncpus; i++) {
		sprintf(tmp, "%s/%s%d.produced", dirname, basename, i);
		produced_file[i] = open(tmp, O_RDONLY);
		if (produced_file[i] < 0) {
			printf("Couldn't open control file %s\n", tmp);
			goto fail;
		}
	}

	for (i = 0; i < ncpus; i++) {
		sprintf(tmp, "%s/%s%d.consumed", dirname, basename, i);
		consumed_file[i] = open(tmp, O_RDWR);
		if (consumed_file[i] < 0) {
			printf("Couldn't open control file %s\n", tmp);
			goto fail;
		}
	}

	return 0;
fail:
	close_control_files();
	return -1;
}

/**
 *	open_cpu_files - open and mmap buffer and create output file for a cpu
 */
static int open_cpu_files(int cpu, const char *dirname, const char *basename,
			  const char *out_basename)
{
	size_t total_bufsize;
	char tmp[4096];

	memset(&status[cpu], 0, sizeof(struct buf_status));

	sprintf(tmp, "%s/%s%d", dirname, basename, cpu);
	relay_file[cpu] = open(tmp, O_RDONLY | O_NONBLOCK);
	if (relay_file[cpu] < 0) {
		printf("Couldn't open relay file %s: errcode = %s\n",
		       tmp, strerror(errno));
		return -1;
	}

	sprintf(tmp, "%s%d", out_basename, cpu);
	if((out_file[cpu] = open(tmp, O_CREAT | O_RDWR | O_TRUNC, S_IRUSR |
				 S_IWUSR | S_IRGRP | S_IROTH)) < 0) {
		printf("Couldn't open output file %s: errcode = %s\n",
		       tmp, strerror(errno));
		close(relay_file[cpu]);
		return -1;
	}

	total_bufsize = subbuf_size * n_subbufs;
	relay_buffer[cpu] = mmap(NULL, total_bufsize, PROT_READ,
				 MAP_PRIVATE | MAP_POPULATE, relay_file[cpu],
				 0);
	if(relay_buffer[cpu] == MAP_FAILED)
	{
		printf("Couldn't mmap relay file, total_bufsize (%ld) = subbuf_size (%ld) * n_subbufs(%ld), error = %s \n", total_bufsize, subbuf_size, n_subbufs, strerror(errno));
		close(relay_file[cpu]);
		close(out_file[cpu]);
		return -1;
	}

	return 0;
}

/**
 *	close_cpu_files - close and munmap buffer and open output file for cpu
 */
static void close_cpu_files(int cpu)
{
	size_t total_bufsize = subbuf_size * n_subbufs;

	munmap(relay_buffer[cpu], total_bufsize);
	close(relay_file[cpu]);
	close(out_file[cpu]);
}

static void close_app_files(void)
{
	int i;

	for (i = 0; i < ncpus; i++)
		close_cpu_files(i);
}

static int open_app_files(void)
{
	int i;

	for (i = 0; i < ncpus; i++) {
		if (open_cpu_files(i, app_dirname, percpu_basename,
				   percpu_out_basename) < 0) {
			control_write(app_dirname, "enabled", 0);
			control_write(app_dirname, "create", 0);
			return -1;
		}
	}

	return 0;
}

int main(int argc, char **argv)
{
	extern char *optarg;
	extern int optopt;
	int i, c, signal;
	size_t opt_subbuf_size = 0;
	size_t opt_n_subbufs = 0;
	sigset_t signals;

	pthread_mutex_init(&processing_mutex, NULL);

	sigemptyset(&signals);
	sigaddset(&signals, SIGINT);
	sigaddset(&signals, SIGTERM);
	pthread_sigmask(SIG_BLOCK, &signals, NULL);

	while ((c = getopt(argc, argv, "b:n:")) != -1) {
		switch (c) {
		case 'b':
			opt_subbuf_size = (unsigned)atoi(optarg);
			if (!opt_subbuf_size)
				usage();
			break;
		case 'n':
			opt_n_subbufs = (unsigned)atoi(optarg);
			if (!opt_n_subbufs)
				usage();
			break;
		case '?':
			printf("Unknown option -%c\n", optopt);
			usage();
			break;
		default:
			break;
		}
	}

	if ((opt_n_subbufs && !opt_subbuf_size) ||
	    (!opt_n_subbufs && opt_subbuf_size))
		usage();

	if (opt_n_subbufs && opt_n_subbufs) {
		subbuf_size = opt_subbuf_size;
		n_subbufs = opt_n_subbufs;
	}

	ncpus = sysconf(_SC_NPROCESSORS_ONLN);

	control_write(app_dirname, "subbuf_size", subbuf_size);
	control_write(app_dirname, "n_subbufs", n_subbufs);
	/* disable logging in case we exited badly in a previous run */
	control_write(app_dirname, "enabled", 0);
	fprintf(stderr, "control_write: create\n");

	control_write(app_dirname, "create", 1);

	if (open_app_files())
		return -1;

	if (open_control_files(app_dirname, percpu_basename)) {
		close_app_files();
		return -1;
	}

	if (create_percpu_threads()) {
		close_control_files();
		close_app_files();
		return -1;
	}

	control_write(app_dirname, "enabled", 1);

	printf("Creating channel with %lu sub-buffers of size %lu.\n",
	       n_subbufs, subbuf_size);
	printf("Logging... Press Control-C to stop.\n");

	sigemptyset(&signals);
	sigaddset(&signals, SIGINT);
	sigaddset(&signals, SIGTERM);

	while (sigwait(&signals, &signal) == 0) {
		switch(signal) {
		case SIGINT:
		case SIGTERM:
			control_write(app_dirname, "enabled", 0);
			kill_percpu_threads(ncpus);
			while(1) {
				pthread_mutex_lock(&processing_mutex);
				if (!processing) {
					pthread_mutex_unlock(&processing_mutex);
					break;
				}
				pthread_mutex_unlock(&processing_mutex);
			}
			for (i = 0; i < ncpus; i++)
				check_buffer(i);
			summarize();
			close_control_files();
			close_app_files();
			control_write(app_dirname, "create", 0);
			exit(0);
		}
	}
}
