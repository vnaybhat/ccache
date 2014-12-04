/*	$NetBSD */

/*-
 * This code has been donated to The NetBSD Foundation by the Author.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
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

/* System headers */
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/compcache.h>
#include <sys/device.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/param.h>
#include <sys/proc.h>
#include <sys/systm.h>
#include <sys/vmem.h>

/* ZLIB compression libraries */
#include <net/zlib.h>

/* Omnipotent UVM libraries */
#include <uvm/uvm.h>

/* Our compressed cache, for now we support only 
   one compressed cache device */
static compcache_t *ccache;

/* Device driver functions exposed to the outside world */
void compcacheattach(int num);
int compcacheopen(dev_t device, int flags, int fmt, struct lwp *process);
int compcacheclose(dev_t device, int flags, int fmt, struct lwp *process);
int compcacheioctl(dev_t device, u_long command, void* data,
    int flags, struct lwp *process);
int compcachesize(dev_t dev);
void compcachestrategy(struct buf *bp);

/* Static helpers - cache initialization */
static void cache_destroy(compcache_t *cache);
static int cache_init(compcache_t *cache);
static void compressor_destroy(compcache_t *cache);
static int compressor_init(compcache_t *cache);

/* Static helpers - compression related */
static int decomppages(struct buf *bp);
static int comppages(struct buf *bp);

/* Static helpers - cache memory management */
static void ccache_free(cnode_t *cnode);
static char *ccache_alloc(compcache_t *ccacahe, size_t size);

/* Static helpers - compressor memory management */
static void  def_zfree(voidpf opaque, voidpf ptr);
static voidpf def_zcalloc(voidpf opaque, unsigned items, unsigned size);

/* Static helpers - Generic helpers */
static int is_zero(char *data);

/* Character device switch - is this really needed? */
const struct cdevsw compcache_cdevsw = {
        .d_open = compcacheopen,
        .d_close = compcacheclose,
        .d_read = noread,
        .d_write = nowrite,
        .d_ioctl = compcacheioctl,
        .d_stop = nostop,
        .d_tty = notty,
        .d_poll = nopoll,
        .d_mmap = nommap,
        .d_kqfilter = nokqfilter,
        .d_flag = D_OTHER,
};

/* Block device switch */
const struct bdevsw compcache_bdevsw = {
        .d_open = compcacheopen,
        .d_close = compcacheclose,
        .d_strategy = compcachestrategy,
        .d_ioctl = compcacheioctl,
        .d_dump = nodump,
        .d_psize = compcachesize,
        .d_discard = nodiscard,
        .d_flag = D_DISK | D_MPSAFE
};


/****************** Functions exposed to the outside world ******************/
void
compcacheattach(int num)
{
	  /* nothing to do for compcache, this is where resources that
	     need to be allocated/initialised before open is called
	     can be set up. But we allocate resources only on open
             since we donot want to tie up memory for compressed cache
             unless the user installs it */

          printf("In compcache attach.\n");
          printf("In compcache attach.\n");
          printf("In compcache attach.\n");
          printf("In compcache attach.\n");

          return;
}


/*
 * Handle an open request on the device.
 * We initialize the compressed cache state here
 */
int
compcacheopen(dev_t device, int flags, int fmt, struct lwp *process)
{
        /* Allocate memory for cache structure and initialize */
        ccache = malloc(sizeof(compcache_t), M_DEVBUF, M_NOWAIT);
        if(!ccache) {
                goto err;
        }
        memset(ccache, 0, sizeof(compcache_t));

        if(!cache_init(ccache)) {
            goto err;
        }

        if(!compressor_init(ccache)) {
            goto err;
        }

        mutex_init(&ccache->cmut, MUTEX_DEFAULT, IPL_NONE);
        return 0;

err:
        printf("Failed to initialize compressed cache\n");
        if(!ccache)
                return ENOMEM;

        if(ccache->stagearea)
                cache_destroy(ccache);

        if(ccache->compressarea)
                compressor_destroy(ccache);

        free(ccache, M_DEVBUF);
        ccache = NULL;

        return ENOMEM;
}

/* 
 * Returns thenumber of blocks in swap device, there are 8 blocks
 * per page in our cache device 
 */
int 
compcachesize(dev_t device)
{
        return NUMPAGES << BLKS_PER_PAGE_SHIFT;
}

/*
 * Handle the close request for the device.
 * Here we have to deallocate all the resources allocated for our
 * compressed swap. This mainly deasl with destroying all the arena's
 * and then freeing up the memory related to that arena. Other resources
 * like mutexes should also be destroyed.
 */
int
compcacheclose(dev_t device, int flags, int fmt, struct lwp *process)
{
        cache_destroy(ccache);
        compressor_destroy(ccache);

        free(ccache, M_DEVBUF);
        ccache = NULL;
        
	return 0; /* again this always succeeds */
}

/*
 * Handle the ioctl for the device.
 * We are not provideing a character device interface to oyr swap space
 * Hence this is not needed
 * XXX: Once the cache device is fully functional and we are ready to analyze
 * performce, we can use this as a user space interface to get the swap 
 * statistics for performance analysis.
 */
int
compcacheioctl(dev_t device, u_long command, void* data, int flags,
	      struct lwp *process)
{
	return 0;
}

/*
 * The strategy routine. Mainly deals with calling appropriate routines.
 * For read requet, calls decompress page routine.
 * For write request, calls compress pages routine
 */
void
compcachestrategy(struct buf *bp)
{
        KASSERT(bp != NULL);

        if ((bp->b_flags & B_READ)) {
                if(decomppages(bp) < 0)
                    bp->b_error = ENOMEM;
        }
        else if((bp->b_flags & B_READ) == 0) {
                if(comppages(bp) < 0)
                    bp->b_error = ENOMEM;
        }
        else
                bp->b_error = ENOMEM;

        biodone(bp);
        return;
}

/****************** Functions exposed to the outside world ******************/
/****************************************************************************/
/****************************************************************************/

/******************** Compression/Decompression related *********************/
/*
 * The decompress routine. Called on a read request on this device.
 * Identifies if a page is zero-page, non-compressed page or a compressed-page
 * and handles it appropriately.
 * A zero-page is one which is not stored because the entire page content is
 * always zero.
 * A non-compressed page is one which is stored uncompressed mainly because
 * the zlib library could not compress this page (in future we can think of 
 * storing those pages whose compression ratio is below a certain threshold 
 * also as a non-compressed page.
 * A compressed page is one which was previously compressed by zlib. Use the
 * page metadata to decompress now.
 */
static int
decomppages(struct buf *bp)
{
        z_stream strm; /* Is it ok for this to go in stack? XXX */
        unsigned npages, curpage, i;
        char *input, *output;

        KASSERT(bp != NULL);

        mutex_enter(&ccache->cmut);
        npages = bp->b_bcount >> PAGE_SHIFT;
        i = (bp->b_blkno >> BLKS_PER_PAGE_SHIFT) - 1;
        curpage = 0;

        while(curpage < npages) {
                KASSERT(i < NUMPAGES);

                output = (char *) ((char *)bp->b_data +
                    (PAGE_SIZE * curpage));
                input = ccache->index[i].cache_addr;

                if(ccache->index[i].node_size == PAGE_SIZE) 
                        /* The page was stored uncompressed */
                        memcpy(output, input, PAGE_SIZE);
                else if(ccache->index[i].zero_page == 1)
                        /* This is a zero page, nothing was stored */
                        memset(output, 0, PAGE_SIZE);
                else {
                        /* Initialize zlib stream */
                        memset(&strm, 0, sizeof(z_stream));
                        strm.zalloc = def_zcalloc;
                        strm.zfree  = def_zfree;
                        strm.opaque = Z_NULL;
                        strm.avail_in = 0;
                        strm.next_in = Z_NULL;

                        if(inflateInit(&strm) != Z_OK) {
                                /* Ideally this should never happen */
                                printf("inflate init error..\n");
                                mutex_exit(&ccache->cmut);

                                /* Only for debugging, 
                                   will be removed later */
                                KASSERT(0);
                                return -1;
                    }

                    strm.avail_in = ccache->index[i].node_size;
                    strm.next_in = input;
                    strm.avail_out = PAGE_SIZE;
                    strm.next_out = output;

                    int infret = 0;
                    if((infret = inflate(&strm, Z_FINISH)) != Z_STREAM_END) {
                            printf("Something seriously bad happened "
                                   "when inflating. infret = %d\n", (int)infret);
                            inflateEnd(&strm);
                            mutex_exit(&ccache->cmut);

                            /* Only for debugging, 
                               will be removed later */
                            KASSERT(0);
                            return -1; 
                    }
                    inflateEnd(&strm);
                }
                curpage++;
                i++;
        }
                        
        mutex_exit(&ccache->cmut);
        return 0;
}

/*
 * The dcompress routine. Called on a write request on this device.
 * Identifies if a page is zero-page, non-compressable page or a 
 * compresable page and handles it appropriately.
 * A zero-page is one which is not stored because the entire page content is
 * always zero.
 * A non-compressable page is one which is stored uncompressed mainly because
 * the zlib library could not compress this page (in future we can think of 
 * storing those pages whose compression ratio is below a certain threshold 
 * also as a non-compressed page).
 * A compressed page is one which was can be compressed by the zlib library
 * and hence appropriate metadata is also stored along with the compressed 
 * data
 */
static int
comppages(struct buf *bp)
{
        z_stream strm; /* XXX Is it ok for this to go in stack? */
        unsigned npages, curpage, i;
        daddr_t  cur_blkno;
        char *input, *output;
        char *cache_addr;

        KASSERT(bp != NULL);

        npages = bp->b_bcount >> PAGE_SHIFT;
        curpage = 0;
        cur_blkno = bp->b_blkno;
        i = (cur_blkno >> BLKS_PER_PAGE_SHIFT) - 1;

        mutex_enter(&ccache->cmut);

        output = ccache->buffer;
        while(curpage < npages) {
                KASSERT(i < NUMPAGES);

                /* Free up if the block was previously allocated */
                if(ccache->index[i].node_size)
                        ccache_free(&(ccache->index[i]));

                /* Get the address of next input location */
                input = (char *) ((char *)bp->b_data +
                        (PAGE_SIZE * curpage));

                if(is_zero(input)) {
                        ccache->index[i].node_size = 0;
                        ccache->index[i].zero_page = 1;
                        ccache->index[i].cache_addr = NULL;
                        goto done;
                }

                memset(&strm, 0, sizeof(z_stream));
                strm.zalloc = def_zcalloc;
                strm.zfree  = def_zfree;
                strm.opaque = Z_NULL;

                if(deflateInit2(&strm, Z_DEFAULT_COMPRESSION,
                            Z_DEFLATED, 15, 8, 
                            Z_DEFAULT_STRATEGY) != Z_OK) {
                        /* Store uncompressed on init error */
                        cache_addr = ccache_alloc(ccache, PAGE_SIZE);
                        memcpy(cache_addr, input, PAGE_SIZE);
                        ccache->index[i].node_size = PAGE_SIZE;
                        ccache->index[i].cache_addr = cache_addr;
                }
                else {
                        strm.avail_in  = PAGE_SIZE;
                        strm.avail_out = PAGE_SIZE;
                        strm.next_in   = input;
                        strm.next_out  = output;
             
                        if(deflate(&strm, Z_FINISH) != Z_STREAM_END) {
                                /* No compression, store uncompressed */
                                cache_addr = ccache_alloc(ccache, PAGE_SIZE);
                                memcpy(cache_addr, input, PAGE_SIZE);
                                ccache->index[i].node_size = PAGE_SIZE;
                                ccache->index[i].cache_addr = cache_addr;
                        }
                        else {
                                ccache->index[i].node_size = 
                                    PAGE_SIZE - strm.avail_out; 
                                cache_addr = ccache_alloc(ccache, 
                                    ccache->index[i].node_size);
                                ccache->index[i].cache_addr = cache_addr;
                                memcpy(cache_addr, output, 
                                    ccache->index[i].node_size);
                        }
                        deflateEnd(&strm);
                }
                ccache->index[i].zero_page = 0;

done:
                ccache->index[i].blkno = cur_blkno;

                cur_blkno += 8;
                curpage++;
                i++;
        }

        mutex_exit(&ccache->cmut);

        return 0;
}
/******************** Compression/Decompression related *********************/
/****************************************************************************/
/****************************************************************************/

/********************** Cache memory management *****************************/
static char *
ccache_alloc(compcache_t *cache, size_t size)
{
        vmem_addr_t cache_addr = (vmem_addr_t)0;
        
        int ret = vmem_alloc(cache->stagearena, size, 
            VM_BESTFIT | VM_NOSLEEP, &cache_addr);

        if(ret != 0) {
            printf("Failed to alloc %d bytes from stageareana.",
                (int)size);
            printf("Allocd in arena: %d, Free in arena: %d, Tot size: %d\n", 
                (int)vmem_size(cache->stagearena, VMEM_ALLOC),
                (int)vmem_size(cache->stagearena, VMEM_FREE),
                (int)vmem_size(cache->stagearena, VMEM_FREE|VMEM_ALLOC));
        }

        /* Ideally should never happen, assert if it does */
        KASSERT(ret == 0);

        return (char *)cache_addr;

}

static void
ccache_free(cnode_t *cnode)
{
        if(!cnode)
            return;

        vmem_free(ccache->stagearena, (vmem_addr_t)cnode->cache_addr, 
            cnode->node_size);

        cnode->node_size = 0;
        cnode->cache_addr = NULL;

        return;
}
/********************** Cache memory management *****************************/
/****************************************************************************/
/****************************************************************************/

/************************ Compressor memory management **********************/
static voidpf 
def_zcalloc(voidpf opaque, unsigned items, unsigned size)
{
        vmem_addr_t cache_addr = (vmem_addr_t)0;
     
        /* We request for sizeof(unsigned) more bytes to store the size */
        size = (items * size) + sizeof(unsigned);

        int ret = vmem_alloc(ccache->compressarena, size,
            VM_INSTANTFIT | VM_NOSLEEP, &cache_addr);


        if(ret != 0) {
            printf("Failed to alloc %d bytes from compressareana. "
                   "ret=%d, cache_addr = %p\n", 
                (int)(size*items), (int)ret, (void *)cache_addr);
        }

        /* Should never happen, assert if it does */
        KASSERT(ret == 0);

        /* Store the size, needed later for free */
        *((unsigned *)cache_addr) = size;

        /* Return the correct address back after accounting for 
           the stored size */
        cache_addr = (vmem_addr_t)((unsigned *)cache_addr + 1);

        return (voidpf)cache_addr;
}

static void 
def_zfree(voidpf opaque, voidpf ptr)
{
       /* get the size to be freed */
       unsigned size = *((unsigned *)ptr - 1);

       vmem_free(ccache->compressarena, (vmem_addr_t)((unsigned *)ptr - 1),
	   size);

       return;
}       
/************************ Compressor memory management **********************/
/****************************************************************************/
/****************************************************************************/

/*************************** Cache init destroy *****************************/
static void 
cache_destroy(compcache_t *cache)
{
        int i = 0;

        if(cache->index) {
                for(i = 0; i < NUMPAGES; i++) {
                        if(cache->index[i].node_size) {
                                vmem_free(ccache->stagearena, 
                                    (vmem_addr_t)cache->index[i].cache_addr,
                                    cache->index[i].node_size);
                        }
                 }
                 free(cache->index, M_DEVBUF);
                 ccache->index = NULL;
        }

        if(cache->stagearea) {
                if(cache->stagearena) {
                        vmem_destroy(cache->stagearena);
                        cache->stagearena = NULL;
                }

                uvm_km_free(kernel_map, (vaddr_t) cache->stagearea,
                    (NUMPAGES)*PAGE_SIZE,
                    UVM_KMF_WIRED | UVM_KMF_CANFAIL | UVM_KMF_NOWAIT);
                cache->stagearea = NULL;
        }


        return;
}

static int 
cache_init(compcache_t *cache)
{
        /* Allocate memory for the cache */
        cache->stagearea = (char *) uvm_km_alloc(kernel_map,
            (NUMPAGES)*PAGE_SIZE, PAGE_SIZE,
            UVM_KMF_WIRED | UVM_KMF_CANFAIL | UVM_KMF_NOWAIT);
        if(!cache->stagearea) {
                cache_destroy(cache);
                return 0;
        }
        memset(cache->stagearea, 0, (NUMPAGES)*PAGE_SIZE);

        /* Create the cache arena */
        cache->stagearena = vmem_create("stage_arena",
            (vmem_addr_t)cache->stagearea,
            (NUMPAGES)*PAGE_SIZE, QUANTUM, NULL, NULL, NULL,
            PAGE_SIZE, VM_NOSLEEP, IPL_NONE);
        if(!cache->stagearena) {
                cache_destroy(cache);
                return 0;
        }

        /* This maintains the information of each cached page */
        cache->index = malloc(sizeof(cnode_t) * NUMPAGES,
            M_DEVBUF, M_NOWAIT);
        if(!cache->index) {
                cache_destroy(cache);
                return 0;
        }
        memset(cache->index, 0, sizeof(cnode_t) * NUMPAGES);

        return 1;
}


static void 
compressor_destroy(compcache_t *cache)
{
        if(cache->compressarea) {
                uvm_km_free(kernel_map, (vaddr_t)cache->compressarea,
                    (NUM_COMP_PAGES)*PAGE_SIZE,
                    UVM_KMF_WIRED | UVM_KMF_CANFAIL | UVM_KMF_NOWAIT);
                cache->compressarea = NULL;

                if(cache->compressarena) {
                        vmem_destroy(cache->compressarena);
                        cache->compressarena = NULL;
                }
        }

        return;
}

static int 
compressor_init(compcache_t *cache)
{
        /* Initialize memory needed by compressor */
        cache->compressarea = (char *) uvm_km_alloc(kernel_map,
            (NUM_COMP_PAGES)*PAGE_SIZE, PAGE_SIZE,
            UVM_KMF_WIRED | UVM_KMF_CANFAIL | UVM_KMF_NOWAIT);
        if(!cache->compressarea) {
                return 0;
        }
        memset(cache->compressarea, 0, (NUM_COMP_PAGES)*PAGE_SIZE);

        /* Initialize the compress arena */
        cache->compressarena = vmem_create("compress_arena", 
            (vmem_addr_t)cache->compressarea,
            (NUM_COMP_PAGES)*PAGE_SIZE, QUANTUM, NULL, NULL, NULL,
            PAGE_SIZE, VM_NOSLEEP, IPL_VM);
        if(!cache->compressarena) {
                compressor_destroy(cache);
                return 0;
        }

        return 1;
}
/*************************** Cache init destroy *****************************/
/****************************************************************************/
/****************************************************************************/

/************************* Other static helpers *****************************/
/* Check if the entire page is a zero filled page */
static int
is_zero(char *data)
{
        int i;
        for(i = 0; i < PAGE_SIZE; i++)
                if(data[i] != 0)
                        return 0;

        return 1;
}
/************************* Other static helpers *****************************/
/****************************************************************************/
/****************************************************************************/
