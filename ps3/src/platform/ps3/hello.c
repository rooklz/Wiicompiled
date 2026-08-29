/* hello.c — launch detector.
 *
 * Attempts a file write, then loops forever. The infinite loop is the actual
 * test: it turns "did this process start" into something observable without a
 * file, a screen or a network round-trip. If the app returns to the XMB, it
 * never ran; if it hangs, it ran and the file path is the thing to fix.
 */
#include <sys/process.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

SYS_PROCESS_PARAM(1001, 0x100000)

int main(void)
{
    int fd = open("/dev_hdd0/tmp/wiicompiled-hello.txt",
                  O_CREAT | O_TRUNC | O_WRONLY, S_IRWXU | S_IRWXG | S_IRWXO);
    if (fd >= 0) {
        static const char msg[] = "launched\n";
        write(fd, msg, sizeof msg - 1);
        close(fd);
    }

    /* Hang deliberately. Returning would drop to the XMB and erase the one bit
     * of information this build exists to produce. */
    for (;;) { }
    return 0;
}
