/*	$NetBSD: compcache.h */

/*-
 * This code has been donated to The NetBSD Foundation by the Author.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software withough specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include <sys/param.h>
#include <sys/device.h>
#include <sys/vmem.h>

#ifndef _COMPCACHE_H_
#define _COMPCACHE_H_ 

#define NUMPAGES             (1 << 12)
#define CCACHE_BLOCK_SIZE    512
#define BLKS_PER_PAGE_SHIFT  3
#define QUANTUM              1
#define NUM_COMP_PAGES      100

typedef struct cachenode {
        unsigned    node_size;
        daddr_t     blkno;
        vaddr_t     start;
        char        *cache_addr;
        struct cachenode *next;
} cnode_t;


typedef struct compcache {
        kmutex_t    cmut;
        cnode_t     *index;
        char        *stagearea;
        vmem_t      *stagearena;
        char        *compressarea;
        vmem_t      *compressarena;
        char        buffer[PAGE_SIZE];
} compcache_t;


#endif  /* _COMPCACHE_H_ */

