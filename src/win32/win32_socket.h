#if defined(_WIN32)

#define _AMD64_
#define WIN32_LEAN_AND_MEAN 
#include <windows.h>

#include <stdint.h>

#define _USE_W32_SOCKETS 1

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4255 4668 4820)
#endif

#include <io.h>
#include <winsock2.h>

#if defined(_MSC_VER)
#pragma comment(lib, "ws2_32.lib")
#endif

#include "./unistd/unistd.h"

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

/*
 * winsock doesn't feature poll(), so there is a version implemented in terms of select() in win32_socket.c.
 * The following definitions are copied from linux man pages.
 * A poll() macro is defined to call the version in win32_socket.c.
 */
#if !defined(_WIN32_WINNT) || (_WIN32_WINNT < 0x0600)

#define POLLIN      0x0001    /* There is data to read */
#define POLLPRI     0x0002    /* There is urgent data to read */
#define POLLOUT     0x0004    /* Writing now will not block */
#define POLLERR     0x0008    /* Error condition */
#define POLLHUP     0x0010    /* Hung up */
#define POLLNVAL    0x0020    /* Invalid request: fd not open */
struct pollfd {
    SOCKET fd;        /* file descriptor */
    int16_t events;     /* requested events */
    int16_t revents;    /* returned events */
};
#endif
#define poll(x, y, z)     win32_poll(x, y, z)

/* 
 * These wrappers do nothing special except set the global errno variable if an error occurs
 * (winsock doesn't do this by default).
 * They set errno to unix-like values (i.e. WSAEWOULDBLOCK is mapped to EAGAIN),
 * so code outside of this file "shouldn't" have to worry about winsock specific error handling.
 */
#define socket(x, y, z)   win32_socket(x, y, z)
#define connect(x, y, z)  win32_connect(x, y, z)
#define accept(x, y, z)   win32_accept(x, y, z)
#define shutdown(x, y)    win32_shutdown(x, y)
#define read(x, y, z)     win32_read_socket(x, y, z)
#define write(x, y, z)    win32_write_socket(x, y, z)

/* Winsock uses int32_t instead of the usual socklen_t */
typedef int32_t socklen_t;

int32_t win32_poll(struct pollfd *, uint32_t, int32_t);
SOCKET  win32_socket(int32_t, int32_t, int32_t);
int32_t win32_connect(SOCKET, struct sockaddr*, socklen_t);
SOCKET  win32_accept(SOCKET, struct sockaddr*, socklen_t *);
int32_t win32_shutdown(SOCKET, int32_t);
int32_t win32_close_socket(SOCKET fd);

#define strtok_r(x, y, z) win32_strtok_r(x, y, z)
#define strsep(x,y)       win32_strsep(x,y)

char *win32_strtok_r(char *s, const char *delim, char **lasts);
char *win32_strsep(char **stringp, const char *delim);

ssize_t win32_read_socket(SOCKET fd, void *buf, int32_t n);
ssize_t win32_write_socket(SOCKET fd, void *buf, int32_t n);

#endif // defined(_WIN32)
