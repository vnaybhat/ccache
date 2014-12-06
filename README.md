ccache
======

A compressed in-memory swap device for NetBSD

Design:
A block device (pseudo-device) backed by kernel reserved memory is created. 
This device is installed as a swap device (usually with priority 0).
This block device is used as a staging area for the compressed pages. The
read (decompress)/write (compress) interface itself is provided through VFS read/write semantics.

Here is the basic flow:
1. User creates a device node the usual way using mknode
2. Swap device is installed the usual way using swapctl
3. For now, the number of pages reserved for the compressed swap device is fixed to 4096 pages (can think of configuring to a certain %age of the memory available, uvm structs allow us to get this figure)
4. The compression algorithm used is zlib (which was already imported into netbsd kernel source)
5. On device open, I use uvm_km_alloc to reserve pages for compressed cache. Also, the compressor (zlib) requires memory for its internal use and so I reserve 100 pages for zlib's internal use. 
6. I also create a vmem arena (using vmen_create) to manage cache memory and  compressor memory.
7. On swap-out request, vmem_alloc is used to allocate required memory (after compression). A per-page metadata is maintained which records the memory address and compressed size (among other things). This metadata is indexed by the page number.
8. On swap-in request, the information is fetched from the metadata and the page is decompressed and handed over to the swapper.
