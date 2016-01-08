<<<<<<< HEAD
/**
******************************************************************************
* @file    main.c
* @author  System LAB
* @version V1.0.0
* @date    17-June-2015
* @brief   source file  
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
=======
/*
 * Copyright (c) 2012, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */
>>>>>>> refs/remotes/contiki-os/master
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include <sys/stat.h>
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
int _lseek (int file,
	int ptr,
	int dir)
=======
int
_lseek(int file,
       int ptr,
       int dir)
>>>>>>> refs/remotes/contiki-os/master
{
  return 0;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
int _close (int file)
=======
int
_close(int file)
>>>>>>> refs/remotes/contiki-os/master
{
  return -1;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
void _exit (int n)
{
  /* FIXME: return code is thrown away.  */
  while(1);
}
/*---------------------------------------------------------------------------*/
int _kill (int n, int m)
{
   return -1;
}
/*---------------------------------------------------------------------------*/
int _fstat(int file, struct stat *st)
=======
void
_exit(int n)
{
  /* FIXME: return code is thrown away.  */
  while(1) ;
}
/*---------------------------------------------------------------------------*/
int
_kill(int n, int m)
{
  return -1;
}
/*---------------------------------------------------------------------------*/
int
_fstat(int file, struct stat *st)
>>>>>>> refs/remotes/contiki-os/master
{
  st->st_mode = S_IFCHR;
  return 0;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
int _isatty (int fd)
=======
int
_isatty(int fd)
>>>>>>> refs/remotes/contiki-os/master
{
  return 1;
  fd = fd;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
int _getpid	(int n)
{
   return -1;
}
/*---------------------------------------------------------------------------*/
int _open (const char * path, int flags, ...)
=======
int
_getpid(int n)
{
  return -1;
}
/*---------------------------------------------------------------------------*/
int
_open(const char *path, int flags, ...)
>>>>>>> refs/remotes/contiki-os/master
{
  return -1;
}
/*---------------------------------------------------------------------------*/
