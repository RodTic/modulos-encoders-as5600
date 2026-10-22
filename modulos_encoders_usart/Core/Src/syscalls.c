/* syscalls.c - funciones de apoyo de newlib (la libreria C).
 * El proyecto no usa printf ni archivos, pero la libreria las referencia desde stdio y,
 * sin ellas, el enlazador da 4 avisos ("_close is not implemented...") y CubeIDE marca
 * "Build Failed". Son debiles: si algun dia rediriges printf, defines tu _write y gana la tuya. */
#include <sys/stat.h>

__attribute__((weak)) int _close(int fd)                     { (void)fd; return -1; }
__attribute__((weak)) int _lseek(int fd, int ptr, int dir)   { (void)fd; (void)ptr; (void)dir; return 0; }
__attribute__((weak)) int _read(int fd, char *ptr, int len)  { (void)fd; (void)ptr; (void)len; return 0; }
__attribute__((weak)) int _write(int fd, char *ptr, int len) { (void)fd; (void)ptr; return len; }
__attribute__((weak)) int _fstat(int fd, struct stat *st)    { (void)fd; st->st_mode = S_IFCHR; return 0; }
__attribute__((weak)) int _isatty(int fd)                    { (void)fd; return 1; }
