/*
 * Copyright (c) 2000, 2001, 2002, 2003, 2004, 2005, 2008, 2009
 *	The President and Fellows of Harvard College.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include "opt-A3.h"
#include "opt-A2.h"
#include <types.h>
#include <kern/errno.h>
#include <lib.h>
#include <spl.h>
#include <spinlock.h>
#include <proc.h>
#include <current.h>
#include <mips/tlb.h>
#include <addrspace.h>
#include <vm.h>

#if OPT_A2

#include <copyinout.h>
#include <limits.h>

#endif

#if OPT_A3

#include <mips/trapframe.h>
#include <synch.h>
#include <mips/vm.h>

// Core-map 
static struct coremap_entry *core_map;

// Lock for core-map
static struct lock *core_map_lock;

// Total number of frames available
static unsigned long num_of_frames = 0;

// To check whether the vm has been initialized
static bool vm_init = false;

// Save the base paddress that returned by ram_getsize()
// In order to calculate the paddress for each fram
static paddr_t BASE;

#endif
/*
 * Dumb MIPS-only "VM system" that is intended to only be just barely
 * enough to struggle off the ground.
 */

/* under dumbvm, always have 48k of user stack */
#define DUMBVM_STACKPAGES    12

/*
 * Wrap rma_stealmem in a spinlock.
 */
static struct spinlock stealmem_lock = SPINLOCK_INITIALIZER;

void
vm_bootstrap(void)
{

#if OPT_A3

	// spinlock_acquire(&stealmem_lock);

	if (core_map_lock == NULL) {
		core_map_lock = lock_create("core_map_lock");
		if (core_map_lock == NULL) {
			kprintf("fail to create core_map_lock\n");
			return;
		}
	} 

	lock_acquire(core_map_lock);

	// spinlock_release(&stealmem_lock);

	paddr_t lo, hi;

	// Get all available memory
	ram_getsize(&lo,&hi);

	// Initialize the BASE address that returned by ram_getsize()
	BASE = lo;

	// Current num of available frames that includes frames to store core_map (truncation)
	num_of_frames = (hi - lo) / PAGE_SIZE;

	// Calculate the number of frames that needed to store the entire core_map
	unsigned long num_of_frame_core_map = (ROUNDUP((sizeof(struct coremap_entry))*num_of_frames,PAGE_SIZE))/PAGE_SIZE;

	// Place the core_map at the beginning of the memory that returned by ram_getsize
	core_map = (struct coremap_entry *)PADDR_TO_KVADDR(lo);

	for (unsigned long i = 0; i < num_of_frames; ++i) {
		// Initialize each field of the coremap_entry structure
		(core_map+i)->addr_base = BASE + PAGE_SIZE*i;

		if (i < num_of_frame_core_map) { // mark as invalid since we stored core_map here
			(core_map+i)->num_frame = 1;			
		} else {
			(core_map+i)->num_frame = 0;
		}
	}

	// We have initialized the vm system, so mark it as true
	vm_init = true;

	lock_release(core_map_lock);

#endif	
}

static
paddr_t
getppages(unsigned long npages)
{
	paddr_t addr;

	// spinlock_acquire(&stealmem_lock);
		
#if OPT_A3

	(void)stealmem_lock;

	// spinlock_release(&stealmem_lock);

	if (vm_init) { // our vm system has been initialized, need to take over memory management

		if (core_map_lock == NULL) {
			core_map_lock = lock_create("core_map_lock");
			if (core_map_lock == NULL) {
				kprintf("fail to create core_map_lock\n");
				return -1;
			}
		}

		lock_acquire(core_map_lock);

		unsigned long count = npages;

		for (unsigned long i = 0; i < num_of_frames; ++i) {			

			if (0 == (core_map+i)->num_frame) { // found an unused frame

				--count;

				for (unsigned long j = i+1; j < i+npages; ++j) {

					if (0 == count) { // found it
						break;
					}

					if (0 != (core_map+j)->num_frame) { // not enough contiguous memory to alloc
						i = j+i; // go to next "might available" frame to check
						count = npages; // reset the counter
						break;
					}

					--count;
				}
			}

			if (0 == count) { // found feasible contiguous memory
				
				(core_map+i)->num_frame = npages; // mark the first frame as the npages
				addr = (core_map+i)->addr_base; // set the return addr to be the corresponding paddr

				for (unsigned long k = i+1; k < i+npages; ++k) {
					(core_map+k)->num_frame = -1; // update following frames
				}	

				break;
			}

		}

/*
// Trace the core_map entry
for (unsigned long i = 0; i < num_of_frames; ++i) {
	kprintf("%ld ",(core_map+i)->num_frame);
}
kprintf("\n");
kprintf("\n");
kprintf("\n");
*/
		if (count > 0) { // no enough memory
			kprintf("SHOULD NOT COME HERE\n");
			panic("OUT OF MEMORY!!!!!\n");
		}

		lock_release(core_map_lock);
	
	} else {
		addr = ram_stealmem(npages);
	}


#endif	

	// spinlock_release(&stealmem_lock);
	
	return addr;
}

/* Allocate/free some kernel-space virtual pages */
vaddr_t 
alloc_kpages(int npages)
{
	paddr_t pa;
	pa = getppages(npages);
	if (pa==0) {
		return 0;
	}
	return PADDR_TO_KVADDR(pa);
}

void 
free_kpages(vaddr_t addr)
{
	/* nothing - leak the memory. */

	// (void)addr;

#if OPT_A3

	if (core_map_lock == NULL) {
		core_map_lock = lock_create("core_map_lock");
		if (core_map_lock == NULL) {
			kprintf("fail to create core_map_lock\n");
			return;
		}
	}

	lock_acquire(core_map_lock);

	paddr_t paddr = addr - MIPS_KSEG0;
	KASSERT(0 == ((paddr - BASE)%PAGE_SIZE)); // must be a valid paddr

	unsigned long index = (paddr - BASE)/PAGE_SIZE; // get the corresponding entry

	KASSERT(index < num_of_frames); // must be accessible in the core_map

	long frames_need_to_free = (core_map+index)->num_frame;

	KASSERT(frames_need_to_free > 0);

	for (unsigned long i = index; i < index+frames_need_to_free; ++i) {
		(core_map+i)->num_frame = 0;
	}

	lock_release(core_map_lock);

#endif

}

void
vm_tlbshootdown_all(void)
{
	panic("dumbvm tried to do tlb shootdown?!\n");
}

void
vm_tlbshootdown(const struct tlbshootdown *ts)
{
	(void)ts;
	panic("dumbvm tried to do tlb shootdown?!\n");
}

int
vm_fault(int faulttype, vaddr_t faultaddress)
{
	vaddr_t vbase1, vtop1, vbase2, vtop2, stackbase, stacktop;
	paddr_t paddr;
	int i;
	uint32_t ehi, elo;
	struct addrspace *as;
	int spl;

	faultaddress &= PAGE_FRAME;

	DEBUG(DB_VM, "dumbvm: fault: 0x%x\n", faultaddress);

	switch (faulttype) {
	    case VM_FAULT_READONLY:
		/* We always create pages read-write, so we can't get this */
		// panic("dumbvm: got VM_FAULT_READONLY\n");
#if OPT_A3
	    	/* EX_MOD means TLB Modify (write to read-only page) */
	    	return EX_MOD;
	    	// Instead of calling kill_curthread directly, return an error would trigger mips_trap
	    	// kill_curthread((vaddr_t)NULL,EX_MOD,(vaddr_t)NULL);
#endif
	    case VM_FAULT_READ:
	    case VM_FAULT_WRITE:
		break;
	    default:
		return EINVAL;
	}

	if (curproc == NULL) {
		/*
		 * No process. This is probably a kernel fault early
		 * in boot. Return EFAULT so as to panic instead of
		 * getting into an infinite faulting loop.
		 */
		return EFAULT;
	}

	as = curproc_getas();
	if (as == NULL) {
		/*
		 * No address space set up. This is probably also a
		 * kernel fault early in boot.
		 */
		return EFAULT;
	}

#if OPT_A3

	/* Assert that the address space has been set up properly. */
	KASSERT(as->as_vbase1 != 0);
	KASSERT(as->as_pbase1 != NULL);
	KASSERT(as->as_npages1 != 0);
	KASSERT(as->as_vbase2 != 0);
	KASSERT(as->as_pbase2 != NULL);
	KASSERT(as->as_npages2 != 0);
	KASSERT(as->as_stackpbase != NULL);
	KASSERT((as->as_vbase1 & PAGE_FRAME) == as->as_vbase1);
	
	for (size_t i = 0; i < as->as_npages1; ++i) {
		KASSERT((((as->as_pbase1)+i)->addr_base & PAGE_FRAME) == ((as->as_pbase1)+i)->addr_base);
	}
	

	KASSERT((as->as_vbase2 & PAGE_FRAME) == as->as_vbase2);
	

	for (size_t i = 0; i < as->as_npages2; ++i) {
		KASSERT((((as->as_pbase2)+i)->addr_base & PAGE_FRAME) == ((as->as_pbase2)+i)->addr_base);
	}

	for (size_t i = 0; i < DUMBVM_STACKPAGES; ++i) {
		KASSERT((((as->as_stackpbase)+i)->addr_base & PAGE_FRAME) == ((as->as_stackpbase)+i)->addr_base);
	}

	vbase1 = as->as_vbase1;
	vtop1 = vbase1 + as->as_npages1 * PAGE_SIZE;
	vbase2 = as->as_vbase2;
	vtop2 = vbase2 + as->as_npages2 * PAGE_SIZE;
	stackbase = USERSTACK - DUMBVM_STACKPAGES * PAGE_SIZE;
	stacktop = USERSTACK;

	if (faultaddress >= vbase1 && faultaddress < vtop1) {
		size_t page_number = (faultaddress - vbase1)/PAGE_SIZE;
		paddr = ((as->as_pbase1)+page_number)->addr_base + (faultaddress - vbase1)%PAGE_SIZE;
	}
	else if (faultaddress >= vbase2 && faultaddress < vtop2) {
		size_t page_number = (faultaddress - vbase2)/PAGE_SIZE;
		paddr = ((as->as_pbase2)+page_number)->addr_base + (faultaddress - vbase2)%PAGE_SIZE;
	}
	else if (faultaddress >= stackbase && faultaddress < stacktop) {
		size_t page_number = (faultaddress - stackbase)/PAGE_SIZE;
		paddr = ((as->as_stackpbase)+page_number)->addr_base + (faultaddress - stackbase)%PAGE_SIZE;
	}
	else {
		return EFAULT;
	}

#else

	/* Assert that the address space has been set up properly. */
	KASSERT(as->as_vbase1 != 0);
	KASSERT(as->as_pbase1 != 0);
	KASSERT(as->as_npages1 != 0);
	KASSERT(as->as_vbase2 != 0);
	KASSERT(as->as_pbase2 != 0);
	KASSERT(as->as_npages2 != 0);
	KASSERT(as->as_stackpbase != 0);
	KASSERT((as->as_vbase1 & PAGE_FRAME) == as->as_vbase1);
	KASSERT((as->as_pbase1 & PAGE_FRAME) == as->as_pbase1);
	KASSERT((as->as_vbase2 & PAGE_FRAME) == as->as_vbase2);
	KASSERT((as->as_pbase2 & PAGE_FRAME) == as->as_pbase2);
	KASSERT((as->as_stackpbase & PAGE_FRAME) == as->as_stackpbase);

	vbase1 = as->as_vbase1;
	vtop1 = vbase1 + as->as_npages1 * PAGE_SIZE;
	vbase2 = as->as_vbase2;
	vtop2 = vbase2 + as->as_npages2 * PAGE_SIZE;
	stackbase = USERSTACK - DUMBVM_STACKPAGES * PAGE_SIZE;
	stacktop = USERSTACK;

	if (faultaddress >= vbase1 && faultaddress < vtop1) {
		paddr = (faultaddress - vbase1) + as->as_pbase1;
	}
	else if (faultaddress >= vbase2 && faultaddress < vtop2) {
		paddr = (faultaddress - vbase2) + as->as_pbase2;
	}
	else if (faultaddress >= stackbase && faultaddress < stacktop) {
		paddr = (faultaddress - stackbase) + as->as_stackpbase;
	}
	else {
		return EFAULT;
	}

#endif

	/* make sure it's page-aligned */
	KASSERT((paddr & PAGE_FRAME) == paddr);

	/* Disable interrupts on this CPU while frobbing the TLB. */
	spl = splhigh();

	for (i=0; i<NUM_TLB; i++) {
		tlb_read(&ehi, &elo, i);
		if (elo & TLBLO_VALID) {
			continue;
		}
		ehi = faultaddress;
		elo = paddr | TLBLO_DIRTY | TLBLO_VALID;

		#if OPT_A3

		if (faultaddress >= vbase1 && faultaddress < vtop1 && as->elf_loaded) {
			elo &= ~TLBLO_DIRTY;
		}

		#endif	
			
		DEBUG(DB_VM, "dumbvm: 0x%x -> 0x%x\n", faultaddress, paddr);
		tlb_write(ehi, elo, i);
		splx(spl);
		return 0;
	}

#if OPT_A3

	// Run out of TLB (full) if the code goes here
	ehi = faultaddress;
	elo = paddr | TLBLO_DIRTY | TLBLO_VALID;
	// Randomly replace an TLB entry	
	tlb_random(ehi,elo);
	splx(spl);
	return 0;
#endif

	kprintf("dumbvm: Ran out of TLB entries - cannot handle page fault\n");
	splx(spl);
	return EFAULT;

}

struct addrspace *
as_create(void)
{
	struct addrspace *as = kmalloc(sizeof(struct addrspace));
	if (as==NULL) {
		return NULL;
	}

#if OPT_A3

	as->as_vbase1 = 0;
	as->as_pbase1 = NULL;
	as->as_npages1 = 0;
	as->as_vbase2 = 0;
	as->as_pbase2 = NULL;
	as->as_npages2 = 0;
	as->as_stackpbase = NULL;
	as->elf_loaded = false;

#else

	as->as_vbase1 = 0;
	as->as_pbase1 = 0;
	as->as_npages1 = 0;
	as->as_vbase2 = 0;
	as->as_pbase2 = 0;
	as->as_npages2 = 0;
	as->as_stackpbase = 0;

#endif

	return as;
}

void
as_destroy(struct addrspace *as)
{

#if OPT_A3

	for (size_t i = 0; i < as->as_npages1; ++i) {
		free_kpages(PADDR_TO_KVADDR(((as->as_pbase1)+i)->addr_base));
	}

	for (size_t i = 0; i < as->as_npages2; ++i) {
		free_kpages(PADDR_TO_KVADDR(((as->as_pbase2)+i)->addr_base));
	}

	for (size_t i = 0; i < DUMBVM_STACKPAGES; ++i) {
		free_kpages(PADDR_TO_KVADDR(((as->as_stackpbase)+i)->addr_base));
	}

	kfree(as->as_pbase1);
	kfree(as->as_pbase2);
	kfree(as->as_stackpbase);

#endif

	kfree(as);
}

void
as_activate(void)
{
	int i, spl;
	struct addrspace *as;

	as = curproc_getas();
#ifdef UW
        /* Kernel threads don't have an address spaces to activate */
#endif
	if (as == NULL) {
		return;
	}

	/* Disable interrupts on this CPU while frobbing the TLB. */
	spl = splhigh();

	for (i=0; i<NUM_TLB; i++) {
		tlb_write(TLBHI_INVALID(i), TLBLO_INVALID(), i);
	}

	splx(spl);
}

void
as_deactivate(void)
{
	/* nothing */
}

int
as_define_region(struct addrspace *as, vaddr_t vaddr, size_t sz,
		 int readable, int writeable, int executable)
{
	size_t npages; 

	/* Align the region. First, the base... */
	sz += vaddr & ~(vaddr_t)PAGE_FRAME;
	vaddr &= PAGE_FRAME;

	/* ...and now the length. */
	sz = (sz + PAGE_SIZE - 1) & PAGE_FRAME;

	npages = sz / PAGE_SIZE;

	/* We don't use these - all pages are read-write */
	(void)readable;
	(void)writeable;
	(void)executable;

	if (as->as_vbase1 == 0) {
		as->as_vbase1 = vaddr;
		as->as_npages1 = npages;
#if OPT_A3		
		as->as_pbase1 = kmalloc(sizeof(struct pagetable_entry)*npages);
#endif		
		return 0;
	}

	if (as->as_vbase2 == 0) {
		as->as_vbase2 = vaddr;
		as->as_npages2 = npages;
#if OPT_A3		
		as->as_pbase2 = kmalloc(sizeof(struct pagetable_entry)*npages);
#endif		
		return 0;
	}

	/*
	 * Support for more than two regions is not available.
	 */
	kprintf("dumbvm: Warning: too many regions\n");
	return EUNIMP;
}

static
void
as_zero_region(paddr_t paddr, unsigned npages)
{
	bzero((void *)PADDR_TO_KVADDR(paddr), npages * PAGE_SIZE);
}

int
as_prepare_load(struct addrspace *as)
{

#if OPT_A3

	KASSERT(as->as_pbase1 != NULL);
	KASSERT(as->as_pbase2 != NULL);
	KASSERT(as->as_stackpbase == NULL);

	for (size_t i = 0; i < as->as_npages1; ++i) {
		((as->as_pbase1)+i)->addr_base = getppages(1);
		if (((as->as_pbase1)+i)->addr_base == 0) {
			return ENOMEM;
		}
		as_zero_region((as->as_pbase1)->addr_base, 1);
	}

	for (size_t i = 0; i < as->as_npages2; ++i) {
		((as->as_pbase2)+i)->addr_base = getppages(1);
		if (((as->as_pbase2)+i)->addr_base == 0) {
			return ENOMEM;
		}
		as_zero_region((as->as_pbase2)->addr_base, 1);
	}

	// Create a page table for stack
	as->as_stackpbase = kmalloc(sizeof(struct pagetable_entry)*DUMBVM_STACKPAGES);

	for (size_t i = 0; i < DUMBVM_STACKPAGES; ++i) {
		((as->as_stackpbase)+i)->addr_base = getppages(1);
		if (((as->as_stackpbase)+i)->addr_base == 0) {
			return ENOMEM;
		}
		as_zero_region((as->as_stackpbase)->addr_base, 1);
	}



#else

	KASSERT(as->as_pbase1 == 0);
	KASSERT(as->as_pbase2 == 0);
	KASSERT(as->as_stackpbase == 0);

	as->as_pbase1 = getppages(as->as_npages1);
	if (as->as_pbase1 == 0) {
		return ENOMEM;
	}

	as->as_pbase2 = getppages(as->as_npages2);
	if (as->as_pbase2 == 0) {
		return ENOMEM;
	}

	as->as_stackpbase = getppages(DUMBVM_STACKPAGES);
	if (as->as_stackpbase == 0) {
		return ENOMEM;
	}

	as_zero_region(as->as_pbase1, as->as_npages1);
	as_zero_region(as->as_pbase2, as->as_npages2);
	as_zero_region(as->as_stackpbase, DUMBVM_STACKPAGES);

#endif

	return 0;
}

int
as_complete_load(struct addrspace *as)
{
#if OPT_A3

	// Load elf finished, set the flag true
	as->elf_loaded = true;
/*
	uint32_t ehi, elo;
	// Flush the TLB with (distinct) invalid TLBHI and invalid TLBLO
	for (int i = 0; i < NUM_TLB; i++) {
		ehi = TLBHI_INVALID(i);
		elo = TLBLO_INVALID();
		tlb_write(ehi,elo,i);
	}
*/
	// as_activate will flush the TLB
	as_activate();

#endif	
	return 0;
}

int
as_define_stack(struct addrspace *as, vaddr_t *stackptr, char **args, unsigned long nargs)
{
	KASSERT(as->as_stackpbase != 0);

#if OPT_A2

	int error;
	// Keep track of total bytes need to be allocated
	unsigned long total_bytes = 0;

	// keep track of the bytes each argument
	unsigned long bytes[nargs];

	for (unsigned long i = 0; i < nargs; ++i) {
		// Align to 8*n bytes

		bytes[i] = ROUNDUP(strlen(args[i])+1,8);

		total_bytes = total_bytes + bytes[i];

		size_t GOT;
		error = copyoutstr((const char *)args[i],(userptr_t)USERSTACK-total_bytes,ARG_MAX,&GOT);
		if (error != 0) {
			return error;
		}
	}

	total_bytes = total_bytes + ROUNDUP(4*(nargs+1),8);

	userptr_t cur_string = (userptr_t)USERSTACK;

	const char *cur_pointer[nargs+1];
	
	for (unsigned long i = 0; i < nargs; ++i) {
		cur_pointer[i] = (char *)cur_string-bytes[i];
		/*
		error = copyin(cur_string-bytes[i],(void *)cur_pointer[i],(size_t)bytes[i]);
		if (error != 0) {
			return error;
		}
		*/
		cur_string = cur_string-bytes[i];
	}
	cur_pointer[nargs] = NULL;

	size_t GOT;
	error = copyoutstr((const char *)cur_pointer,(userptr_t)USERSTACK-total_bytes,ARG_MAX,&GOT);
	if (error != 0) {
		return error;
	}


#endif

	*stackptr = (vaddr_t)((userptr_t)USERSTACK-total_bytes);
	return 0;
}

int
as_copy(struct addrspace *old, struct addrspace **ret)
{
	struct addrspace *new;

	new = as_create();
	if (new==NULL) {
		return ENOMEM;
	}

	new->as_vbase1 = old->as_vbase1;
	new->as_npages1 = old->as_npages1;
	new->as_vbase2 = old->as_vbase2;
	new->as_npages2 = old->as_npages2;

#if OPT_A3

	new->as_pbase1 = kmalloc(sizeof(struct pagetable_entry)*new->as_npages1);
	new->as_pbase2 = kmalloc(sizeof(struct pagetable_entry)*new->as_npages2);

#endif	

	/* (Mis)use as_prepare_load to allocate some physical memory. */
	if (as_prepare_load(new)) {
		as_destroy(new);
		return ENOMEM;
	}

#if OPT_A3

	KASSERT(new->as_pbase1 != NULL);
	KASSERT(new->as_pbase2 != NULL);
	KASSERT(new->as_stackpbase != NULL);

	for (size_t i = 0; i < new->as_npages1; ++i) {
		memmove((void *)PADDR_TO_KVADDR(((new->as_pbase1)+i)->addr_base),
			(const void *)PADDR_TO_KVADDR(((old->as_pbase1)+i)->addr_base),
			PAGE_SIZE);
	}

	for (size_t i = 0; i < new->as_npages2; ++i) {
		memmove((void *)PADDR_TO_KVADDR(((new->as_pbase2)+i)->addr_base),
			(const void *)PADDR_TO_KVADDR(((old->as_pbase2)+i)->addr_base),
			PAGE_SIZE);
	}

	for (size_t i = 0; i < DUMBVM_STACKPAGES; ++i) {
		memmove((void *)PADDR_TO_KVADDR(((new->as_stackpbase)+i)->addr_base),
			(const void *)PADDR_TO_KVADDR(((old->as_stackpbase)+i)->addr_base),
			PAGE_SIZE);
	}

#else

	KASSERT(new->as_pbase1 != 0);
	KASSERT(new->as_pbase2 != 0);
	KASSERT(new->as_stackpbase != 0);

	memmove((void *)PADDR_TO_KVADDR(new->as_pbase1),
		(const void *)PADDR_TO_KVADDR(old->as_pbase1),
		old->as_npages1*PAGE_SIZE);

	memmove((void *)PADDR_TO_KVADDR(new->as_pbase2),
		(const void *)PADDR_TO_KVADDR(old->as_pbase2),
		old->as_npages2*PAGE_SIZE);

	memmove((void *)PADDR_TO_KVADDR(new->as_stackpbase),
		(const void *)PADDR_TO_KVADDR(old->as_stackpbase),
		DUMBVM_STACKPAGES*PAGE_SIZE);

#endif
	
	*ret = new;
	return 0;
}
