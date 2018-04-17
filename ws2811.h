/*
 * ws2811.h
 *
 * Copyright (c) 2014 Jeremy Garff <jer @ jers.net>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     1.  Redistributions of source code must retain the above copyright notice, this list of
 *         conditions and the following disclaimer.
 *     2.  Redistributions in binary form must reproduce the above copyright notice, this list
 *         of conditions and the following disclaimer in the documentation and/or other materials
 *         provided with the distribution.
 *     3.  Neither the name of the owner nor the names of its contributors may be used to endorse
 *         or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef __WS2811_H__
#define __WS2811_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "rpihw.h"
#include "pwm.h"


#define WS2811_TARGET_FREQ                       800000   // Can go as low as 400000

// 4 color R, G, B and W ordering
#define SK6812_STRIP_RGBW                        0x18100800
#define SK6812_STRIP_RBGW                        0x18100008
#define SK6812_STRIP_GRBW                        0x18081000
#define SK6812_STRIP_GBRW                        0x18080010
#define SK6812_STRIP_BRGW                        0x18001008
#define SK6812_STRIP_BGRW                        0x18000810
#define SK6812_SHIFT_WMASK                       0xf0000000

// 3 color R, G and B ordering
#define WS2811_STRIP_RGB                         0x00100800
#define WS2811_STRIP_RBG                         0x00100008
#define WS2811_STRIP_GRB                         0x00081000
#define WS2811_STRIP_GBR                         0x00080010
#define WS2811_STRIP_BRG                         0x00001008
#define WS2811_STRIP_BGR                         0x00000810

// predefined fixed LED types
#define WS2812_STRIP                             WS2811_STRIP_GRB
#define SK6812_STRIP                             WS2811_STRIP_GRB
#define SK6812W_STRIP                            SK6812_STRIP_GRBW

struct ws2811_device;

typedef uint32_t ws2811_led_t;                   //< 0xWWRRGGBB
typedef struct
{
    int gpionum;                                 //< GPIO Pin with PWM alternate function, 0 if unused
    int invert;                                  //< Invert output signal
    int count;                                   //< Number of LEDs, 0 if channel is unused
    int strip_type;                              //< Strip color layout -- one of WS2811_STRIP_xxx constants
    ws2811_led_t *leds;                          //< LED buffers, allocated by driver based on count
    uint8_t brightness;                          //< Brightness value between 0 and 255
    uint8_t wshift;                              //< White shift value
    uint8_t rshift;                              //< Red shift value
    uint8_t gshift;                              //< Green shift value
    uint8_t bshift;                              //< Blue shift value
    uint8_t *gamma;                              //< Gamma correction table
} ws2811_channel_t;


#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <errno.h>
#include <stdbool.h>

typedef struct
{
    uint8_t streaming;         //< Streaming vs udp packet generation
    int  fd;          //< Socket bound
    int  port;
} ws2811_server;

typedef struct
{
    uint64_t render_wait_time;                   //< time in µs before the next render can run
    struct ws2811_device *device;                //< Private data for driver use
    const rpi_hw_t *rpi_hw;                      //< RPI Hardware Information
    uint32_t freq;                               //< Required output frequency
    int dmanum;                                  //< DMA number _not_ already in use
    ws2811_channel_t channel[RPI_PWM_CHANNELS];
} ws2811_t;

#define WS2811_RETURN_STATES(X)                                                             \
            X(0, WS2811_SUCCESS, "Success"),                                                \
            X(-1, WS2811_ERROR_GENERIC, "Generic failure"),                                 \
            X(-2, WS2811_ERROR_OUT_OF_MEMORY, "Out of memory"),                             \
            X(-3, WS2811_ERROR_HW_NOT_SUPPORTED, "Hardware revision is not supported"),     \
            X(-4, WS2811_ERROR_MEM_LOCK, "Memory lock failed"),                             \
            X(-5, WS2811_ERROR_MMAP, "mmap() failed"),                                      \
            X(-6, WS2811_ERROR_MAP_REGISTERS, "Unable to map registers into userspace"),    \
            X(-7, WS2811_ERROR_GPIO_INIT, "Unable to initialize GPIO"),                     \
            X(-8, WS2811_ERROR_PWM_SETUP, "Unable to initialize PWM"),                      \
            X(-9, WS2811_ERROR_MAILBOX_DEVICE, "Failed to create mailbox device"),          \
            X(-10, WS2811_ERROR_DMA, "DMA error"),                                          \
            X(-11, WS2811_ERROR_ILLEGAL_GPIO, "Selected GPIO not possible"),                \
            X(-12, WS2811_ERROR_PCM_SETUP, "Unable to initialize PCM"),                     \
            X(-13, WS2811_ERROR_SPI_SETUP, "Unable to initialize SPI"),                     \
            X(-14, WS2811_ERROR_SPI_TRANSFER, "SPI transfer error")                         \

#define WS2811_RETURN_STATES_ENUM(state, name, str) name = state
#define WS2811_RETURN_STATES_STRING(state, name, str) str

typedef enum {
    WS2811_RETURN_STATES(WS2811_RETURN_STATES_ENUM),

    WS2811_RETURN_STATE_COUNT
} ws2811_return_t;

ws2811_return_t ws2811_init(ws2811_t *ws2811);                         //< Initialize buffers/hardware
void ws2811_fini(ws2811_t *ws2811);                                    //< Tear it all down
ws2811_return_t ws2811_render(ws2811_t *ws2811);                       //< Send LEDs off to hardware
ws2811_return_t ws2811_wait(ws2811_t *ws2811);                         //< Wait for DMA completion
const char * ws2811_get_return_t_str(const ws2811_return_t state);     //< Get string representation of the given return state

// return true if should continue
static inline
bool checkForRequest(ws2811_server *server, ws2811_channel_t* channel, ws2811_t *ws2811) {
  struct sockaddr_in clientaddr; /* client addr */
  socklen_t clientlen = sizeof(clientaddr);

  // Wait for connection
  int childfd = accept(server->fd, (struct sockaddr *) &clientaddr, &clientlen);
  if( childfd < 0 )  {
    if(errno==EAGAIN || errno==EWOULDBLOCK) {
        errno = 0;
        return true;
    }
    perror("Accepting error");
    exit(1);
  }

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;
    if (setsockopt (childfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                    sizeof(timeout)) < 0) {
        perror("Failed to set options");
        exit(1);
    }

  pid_t f = fork();
  if( f != 0 ) {
    close(childfd);
    return true;
  }

  printf("accepting new connection on file descriptor: %d\n", childfd);
  while (1) {
    struct {
        uint16_t offset;
        uint16_t length;
    } info;
    int bytes_read = 0;

    bytes_read = read(childfd, &info, sizeof(info));
    if (bytes_read < 0) {
        perror("Failed to read info");
        goto error;
    }
    if ((unsigned)bytes_read != sizeof(info)) {
        printf("info: didn't read appropriate amount read %d expected %d\n", bytes_read, sizeof(info));
        goto error;
    }

    assert (info.length > 0);

    bytes_read = read(childfd, channel->leds + info.offset, info.length * sizeof(ws2811_led_t));
    if (bytes_read < 0) {
        perror("Failed to read data");
        goto error;
    }
    if ((unsigned)bytes_read != info.length * sizeof(ws2811_led_t)) {
        printf("data: didn't read appropriate amount read %d expected %d\n", bytes_read, sizeof(info));
        goto error;
    }

    ws2811_render(ws2811);
    continue;

    error:;
    close(childfd);
    break;
  }
  exit(0);
  return false;
}

static inline
void server_serve(ws2811_t *ws2811, ws2811_channel_t* channel, uint8_t streaming, int port) {
  assert(streaming);
  ws2811_server server;
  server.streaming = streaming;
  server.port = port;
  server.fd = socket(AF_INET, SOCK_STREAM, 0);
  if( server.fd < 0 ) {
    perror("Socket creation error:");
    exit(1);
  }

  {
    /* setsockopt: Handy debugging trick that lets
     * us rerun the server immediately after we kill it;
     * otherwise we have to wait about 20 secs.
     * Eliminates "ERROR on binding: Address already in use" error.
     */
    int optval = 1;
    setsockopt(server.fd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));
  }

  struct sockaddr_in serveraddr; /* server's addr */
  bzero((char *) &serveraddr, sizeof(serveraddr));

  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

  serveraddr.sin_port = htons((unsigned short)server.port);

  if (bind(server.fd, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0) {
    perror("Binding error:");
    exit(1);
  }

  if (listen(server.fd, SOMAXCONN) < 0) {
    perror("Listen error:");
    exit(1);
  }

  struct timeval timeout;
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;
  if (setsockopt (server.fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                  sizeof(timeout)) < 0) {
    perror("Set timeout error:");
    exit(1);
  }
  while(checkForRequest(&server, channel, ws2811));
}

#ifdef __cplusplus
}
#endif

#endif /* __WS2811_H__ */
