/* A minimal "GameCube program": sum an array, store the result, then branch to
 * address 0 to hand control back to the loader (an illegal instruction there
 * stops execution cleanly). No libc, no globals with initializers beyond the
 * array. */
typedef unsigned int u32;
#define RESULT (*(volatile u32 *)0x80300000u)
static const u32 table[8] = {1,2,3,4,5,6,7,8};
void _start(void)
{
    u32 acc = 0x1234;
    int i;
    for (i = 0; i < 8; i++)
        acc = (acc * 31) + table[i];
    RESULT = acc;
    /* Return to the loader: jump to 0, which is illegal and traps. */
    ((void(*)(void))0)();
}
