/*
 * input.h - keyboard / mouse input
 *
 * USB keyboard and mouse are read through evdev (found automatically and
 * re-scanned once a second, so they can be plugged in at any time). The
 * terminal the game was started from is read too, so it can be driven over
 * the serial console; a terminal only reports key presses, never releases,
 * so those arrive as one-off nudges instead of held keys (w/a/s/d step,
 * q/e and arrows turn, space jumps, f flies, b breaks; Ctrl-C quits).
 */
#ifndef INPUT_H
#define INPUT_H

#include <linux/input.h>
#include <termios.h>
#include <time.h>

struct input {
    int             kbd_fd, mouse_fd;
    time_t          last_scan;
    unsigned char   key[KEY_CNT];   /* evdev keys currently held */
    int             mouse_dx, mouse_dy;

    int             tty;
    struct termios  saved_tio;
    int             saved_flags;
    /* terminal nudges since the last frame */
    int             t_move, t_strafe, t_turn, t_look, t_jump, t_fly, t_break;

    int             quit;
};

void input_open(struct input *in);
void input_poll(struct input *in);     /* call once per frame */
void input_close(struct input *in);

#endif
