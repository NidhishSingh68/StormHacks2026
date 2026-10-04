/*
 * input.c - evdev keyboard and mouse, plus terminal fallback (see input.h)
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include "input.h"

#define BITS_PER_LONG   (8 * sizeof(long))
#define NLONGS(n)       (((n) + BITS_PER_LONG - 1) / BITS_PER_LONG)
#define TEST_BIT(b, a)  (((a)[(b) / BITS_PER_LONG] >> ((b) % BITS_PER_LONG)) & 1)

enum { DEV_KEYBOARD, DEV_MOUSE };

static int is_device(int fd, int kind)
{
    unsigned long ev[NLONGS(EV_MAX + 1)];
    unsigned long bits[NLONGS(KEY_MAX + 1)];

    memset(ev, 0, sizeof(ev));
    memset(bits, 0, sizeof(bits));
    if (ioctl(fd, EVIOCGBIT(0, sizeof(ev)), ev) < 0)
        return 0;

    if (kind == DEV_KEYBOARD) {
        if (!TEST_BIT(EV_KEY, ev) ||
            ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0)
            return 0;
        /* letters and arrows: a real keyboard, not a media-key interface */
        return TEST_BIT(KEY_A, bits) && TEST_BIT(KEY_UP, bits);
    }

    if (!TEST_BIT(EV_REL, ev) ||
        ioctl(fd, EVIOCGBIT(EV_REL, sizeof(bits)), bits) < 0)
        return 0;
    return TEST_BIT(REL_X, bits) && TEST_BIT(REL_Y, bits);
}

static int find_device(int kind)
{
    DIR *dir = opendir("/dev/input");
    struct dirent *de;
    int fd = -1;

    if (!dir)
        return -1;
    while ((de = readdir(dir)) != NULL) {
        char path[300];

        if (strncmp(de->d_name, "event", 5) != 0)
            continue;
        snprintf(path, sizeof(path), "/dev/input/%s", de->d_name);
        fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0)
            continue;
        if (is_device(fd, kind)) {
            /* grab so keys don't also reach the console */
            ioctl(fd, EVIOCGRAB, 1);
            printf("%s: %s\n", kind == DEV_KEYBOARD ? "keyboard" : "mouse", path);
            fflush(stdout);
            break;
        }
        close(fd);
        fd = -1;
    }
    closedir(dir);
    return fd;
}

void input_open(struct input *in)
{
    memset(in, 0, sizeof(*in));
    in->kbd_fd = in->mouse_fd = -1;
    in->last_scan = 0;

    if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &in->saved_tio) == 0) {
        struct termios tio = in->saved_tio;

        tio.c_lflag &= ~(ICANON | ECHO);    /* keep ISIG: Ctrl-C still works */
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) == 0) {
            in->saved_flags = fcntl(STDIN_FILENO, F_GETFL);
            fcntl(STDIN_FILENO, F_SETFL, in->saved_flags | O_NONBLOCK);
            in->tty = 1;
        }
    }
}

void input_close(struct input *in)
{
    if (in->kbd_fd >= 0) {
        ioctl(in->kbd_fd, EVIOCGRAB, 0);
        close(in->kbd_fd);
    }
    if (in->mouse_fd >= 0) {
        ioctl(in->mouse_fd, EVIOCGRAB, 0);
        close(in->mouse_fd);
    }
    if (in->tty) {
        fcntl(STDIN_FILENO, F_SETFL, in->saved_flags);
        tcsetattr(STDIN_FILENO, TCSANOW, &in->saved_tio);
    }
}

/* Read all pending events; returns 0 if the device went away. */
static int drain(struct input *in, int fd)
{
    struct input_event ev[64];
    ssize_t n;
    int i;

    while ((n = read(fd, ev, sizeof(ev))) > 0) {
        for (i = 0; i < n / (ssize_t)sizeof(ev[0]); i++) {
            if (ev[i].type == EV_KEY && ev[i].code < KEY_CNT) {
                in->key[ev[i].code] = (ev[i].value != 0);
            } else if (ev[i].type == EV_REL) {
                if (ev[i].code == REL_X)
                    in->mouse_dx += ev[i].value;
                else if (ev[i].code == REL_Y)
                    in->mouse_dy += ev[i].value;
                else if (ev[i].code == REL_WHEEL)
                    in->wheel += ev[i].value;
            }
        }
    }
    return !(n < 0 && errno != EAGAIN && errno != EWOULDBLOCK);
}

static void poll_terminal(struct input *in)
{
    unsigned char buf[64];
    ssize_t n, i;

    while ((n = read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
        for (i = 0; i < n; i++) {
            unsigned char c = buf[i];

            /* arrow keys: ESC [ A..D (or ESC O A..D) */
            if (c == 0x1B && i + 2 < n && (buf[i + 1] == '[' || buf[i + 1] == 'O')) {
                switch (buf[i + 2]) {
                case 'A': in->t_look++; break;
                case 'B': in->t_look--; break;
                case 'C': in->t_turn++; break;
                case 'D': in->t_turn--; break;
                default: break;
                }
                i += 2;
                continue;
            }
            switch (c) {
            case 'w': case 'W': in->t_move++;   break;
            case 's': case 'S': in->t_move--;   break;
            case 'a': case 'A': in->t_strafe--; break;
            case 'd': case 'D': in->t_strafe++; break;
            case 'q': case 'Q': in->t_turn--;   break;
            case 'e': case 'E': in->t_turn++;   break;
            case ' ':           in->t_jump = 1; break;
            case 'f': case 'F': in->t_fly = 1;  break;
            case 'b': case 'B': in->t_break = 1; break;
            case 'p': case 'P': in->t_place = 1; break;
            case '1': case '2': case '3': case '4': case '5': case '6': case '7':
                in->t_slot = c - '0';
                break;
            default: break;
            }
        }
    }
}

void input_poll(struct input *in)
{
    time_t now = time(NULL);

#ifdef HOST
    /* never grab the development PC's own keyboard and mouse */
    now = in->last_scan;
#endif
    if (now != in->last_scan) {
        in->last_scan = now;
        if (in->kbd_fd < 0)
            in->kbd_fd = find_device(DEV_KEYBOARD);
        if (in->mouse_fd < 0)
            in->mouse_fd = find_device(DEV_MOUSE);
    }

    if (in->kbd_fd >= 0 && !drain(in, in->kbd_fd)) {
        printf("keyboard disconnected\n");
        close(in->kbd_fd);
        in->kbd_fd = -1;
        memset(in->key, 0, sizeof(in->key));
    }
    if (in->mouse_fd >= 0 && !drain(in, in->mouse_fd)) {
        printf("mouse disconnected\n");
        close(in->mouse_fd);
        in->mouse_fd = -1;
    }
    if (in->tty)
        poll_terminal(in);

    if (in->key[KEY_ESC])
        in->quit = 1;
}
