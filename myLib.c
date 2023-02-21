#include "myLib.h"
#include "log.h"
// waiting ms
void jWaitMs(ulong_t ulTime)
{
	struct timeval tv;
	tv.tv_sec = ulTime / 1000;
	tv.tv_usec = (ulTime % 1000) * 1000;
	select(0, NULL, NULL, NULL, &tv);
	return;
}

long timestamp()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void jPselect(unsigned long milliseconds)
{
	// 将毫秒转换为秒和纳秒
	struct timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = (milliseconds % 1000) * 1000000;

	// 使用空的文件描述符集
	fd_set fds;
	FD_ZERO(&fds);

	// 调用pselect()等待
	pselect(0, NULL, NULL, NULL, &ts, NULL);
}
// jGetTick
long jGetTick(void)
{
	struct timeval tv;
	long lSecond, lRtn;
	gettimeofday(&tv, NULL);
	lSecond = ((long)tv.tv_sec) & 0x00ffffff;
	lRtn = (long)(lSecond * 1000) + (long)(tv.tv_usec / 1000);
	return lRtn;
}

int loginput(char *logcontent)
{
	int loginput_fd;
	int logcontent_len;
	int mount_rt;
	// mount_rt = mount("/dev/sda1", "/mnt/sda1", "vfat", MS_MGC_VAL, NULL);
	// if(mount_rt < 0)
	// {
	// 	if(errno != EBUSY)
	// 	{
	// 		perror("mounting USB disk");
	// 		printf("errno : %d\n", errno);
	// 		return -1;
	// 	}
	// }
	logcontent_len = strlen(logcontent);
	loginput_fd = open("DWT-JY.log", O_RDWR | O_CREAT | O_APPEND);
	if (loginput_fd < 0)
	{
		LOG_ERROR("Creat File Fiald %d!\n", loginput_fd);
		return -1;
	}
	if (write(loginput_fd, logcontent, logcontent_len) < 0)
	{
		LOG_ERROR("Write File Fiald %d!\n", loginput_fd);
		return -1;
	}
	close(loginput_fd);
	return 0;
}

void hex_dump(const unsigned char *data, int data_len, int width)
{
    printf("-------------------\n");

    for (int i = 0; i < data_len; i++)
    {
        printf("%02X ", (unsigned char)*(data + i));

        if (((i + 1) % width) == 0)
        {
            printf("\n");
        }
    }

    printf("\n");

    printf("-------------------\n");
}
