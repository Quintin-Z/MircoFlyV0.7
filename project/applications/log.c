#include <rtthread.h>
#include <dfs_posix.h> 


void file_thread()
{
	int fd;
	int size;
	char s[] = "RT-Thread Programmer! \n", buffer[80];
	fd = open("/text.txt", O_WRONLY | O_CREAT,0);

	if (fd >= 0)
	{
		write(fd, s, sizeof(s));
		close(fd);
	}

	fd = open("/text.txt", O_RDONLY,0);
	if (fd >= 0)
	{
		size=read(fd, buffer, sizeof(buffer));
		close(fd);
	}
	rt_kprintf("%s", buffer);
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void file_test()
{
	file_thread();
}
FINSH_FUNCTION_EXPORT(file_test, file_test.)
#endif
