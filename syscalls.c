#include <sys/stat.h>
#include <errno.h>
#include "uart.h"
#undef errno
extern int errno;

void _exit(__attribute__((unused)) int exit_code){
    while(1){}
}

int _close(__attribute__((unused)) int file) {
  return -1;
}

char *__env[1] = { 0 };
char **environ = __env;

int _execve(__attribute__((unused)) char *name, __attribute__((unused)) char **argv, __attribute__((unused)) char **env) {
  errno = ENOMEM;
  return -1;
}

int _fork(void) {
  errno = EAGAIN;
  return -1;
}

int _fstat(__attribute__((unused)) int file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

int _getpid(void) {
  return 1;
}

int _isatty(__attribute__((unused)) int file) {
  return 1;
}

int _kill(__attribute__((unused)) int pid, __attribute__((unused)) int sig) {
  errno = EINVAL;
  return -1;
}

int _link(__attribute__((unused)) char *old, __attribute__((unused)) char *new) {
  errno = EMLINK;
  return -1;
}

int _lseek(__attribute__((unused)) int file, __attribute__((unused)) int ptr, __attribute__((unused)) int dir) {
  return 0;
}

int _open(__attribute__((unused)) const char *name, __attribute__((unused)) int flags, __attribute__((unused)) int mode) {
  return -1;
}

int _read(__attribute__((unused)) int file, __attribute__((unused)) char *ptr, __attribute__((unused)) int len) {
  return 0;
}

register char * stack_ptr asm("sp");
void* _sbrk(int incr) {
  extern char __bss_end__;
  static char *heap_end;
  char *prev_heap_end;
 
  if (heap_end == 0) {
    heap_end = &__bss_end__;
  }
  prev_heap_end = heap_end;
  if (heap_end + incr > stack_ptr) {
    while (1)
    {
        // Heap and stack collision
    }
  }

  heap_end += incr;
  return (void*) prev_heap_end;
}

int _stat(__attribute__((unused)) char *file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

int _times(__attribute__((unused)) struct tms *buf) {
  return -1;
}

int _unlink(__attribute__((unused)) char *name) {
  errno = ENOENT;
  return -1; 
}

int _wait(__attribute__((unused)) int *status) {
  errno = ECHILD;
  return -1;
}

int _write(int file, char *ptr, int len) {
  (void) file;
  
  for (int i = 0; i < len; i++)
  {
    uart_write_char(*ptr++);
  }
  
  return len;
}