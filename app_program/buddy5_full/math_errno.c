#ifdef B5_FULL_TEST
/* Only the controller task uses libm (Buddy 2 sqrtf). Keep its errno slot
 * local instead of pulling newlib's stdio/heap/reentrancy machinery into
 * micro-T-Kernel. This is NOT a general per-task errno implementation.
 * libm's numeric behaviour is unchanged; no application code reads errno. */
int *__wrap___errno(void) {
 static int motion_math_errno;
 return &motion_math_errno;
}
#endif