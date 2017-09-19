#include "opt-A2.h"


#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>


#if OPT_A2

#include <limits.h>
#include <synch.h>
#include <mips/trapframe.h>
#include <array.h>
#include <vfs.h>
#include <kern/fcntl.h>
#include <test.h>

// We have assigned PID_MIN to kproc
pid_t PID_COUNTER = PID_MIN+1;

// Array of PIDs that are available 
extern struct array *PID_available;

// Lock for manage PID and others
extern struct lock *PID_lock;

#endif

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  // (void)exitcode;

#if OPT_A2

  // Set all alive child->parent to NUll
  int num_children = array_num(p->children);
  for (int i = 0; i < num_children; ++i) {
    struct proc *child = (struct proc *)array_get(p->children,i);
    child->parent = NULL;
  }

  // Reuse all pid of exited child
  for (unsigned i = 0; i < array_num(p->child_live); ++i) {
    if (!(int)array_get(p->child_live,i)) {
      array_add(PID_available,array_get(p->children_pid,i),NULL);
      array_remove(p->child_live,i);
      array_remove(p->exitcode,i);
      array_remove(p->children_pid,i);
      --i;
    }
  }

  if (PID_lock == NULL) {
    PID_lock = lock_create("PID_lock");
    if (PID_lock == NULL) {
      return;
    }
  }

  if (PID_available == NULL) {
    PID_available = array_create();
    if (PID_available == NULL) {
      return;
    }
  }
  
  if (p->parent == NULL) {  // parent has exited already
    // kprintf("PPPPPPPID IS :%d  ========PARENT IS DEAD!!!!!!!!!!\n",p->pid);
    //    Don't reuse PID = PID_MIN (2), always assign 2 to kproc
    lock_acquire(PID_lock);
    if (p->pid > PID_MIN) {
      array_add(PID_available,(void *)p->pid,NULL);
    }
    lock_release(PID_lock);

  } else {  // parent has not exited yet  

    lock_acquire(p->parent->proc_lock);

    // Find index of the p in parent's children_pid list
    int index = 0;
    int num_children_pid = array_num(p->parent->children_pid);
    for (int i = 0; i < num_children_pid; ++i) {
      if (p->pid == (pid_t)array_get(p->parent->children_pid,i)) {
        index = i;
        break;
      }
    }

    // Modify child living status to dead in parent's corresponding child_live list
    array_set(p->parent->child_live,index,(void *)0);    

    // Put exitcode in parent's corresponding exitcode list
    array_set(p->parent->exitcode,index,(void *)_MKWAIT_EXIT(exitcode));

    // Remove from the parent's (alive) children list
    int num_sibling = array_num(p->parent->children); 
    for (int i = 0; i < num_sibling; ++i) {
      struct proc *temp = (struct proc *)array_get(p->parent->children,i);
      if (temp == p) {
        array_remove(p->parent->children,i);
        break;
      }
    }

    // Wake up the parent if parent is waiting for exitcode (called waitpid already)
    cv_broadcast(p->proc_cv,p->parent->proc_lock);

    lock_release(p->parent->proc_lock);

  }

  // Make sure all num matches
  /*
  KASSERT(array_num(p->children) == array_num(p->children_pid));
  KASSERT(array_num(p->children_pid) == array_num(p->child_live));
  KASSERT(array_num(p->child_live) == array_num(p->exitcode));
  */


#endif

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
#if OPT_A2
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  *retval = curproc->pid; 
#endif
  return 0;
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */
#if OPT_A2

  struct proc *p = curproc;

  struct proc *child;
  int index = 0;

  lock_acquire(p->proc_lock);

  bool is_my_child = false;
  int num_children_pid = array_num(p->children_pid);
  for (int i = 0; i < num_children_pid; ++i) {
    if ((pid_t)array_get(p->children_pid,i) == pid) {
      is_my_child = true;
      index = i;
      break;
    }
  }

  if (!is_my_child) {
    lock_release(p->proc_lock);
    *retval = -1;
    return ECHILD; // The pid argument named a process that the current process was not interested in or that has not yet exited.
  } else {

    int num_children = array_num(p->children);
    for (int i = 0; i < num_children; ++i) { 
      struct proc *temp = (struct proc *)array_get(p->children,i);
      if (pid == temp->pid) {
        child = temp;
        break;
      }
    }

    // Get exitcode until child has exited
    while ((int)array_get(p->child_live,index)) {
      // kprintf("test!!!!\n");
      cv_wait(child->proc_cv,p->proc_lock);
    }

    exitstatus = (int)array_get(p->exitcode,index);

    array_remove(p->children_pid,index);
    array_remove(p->exitcode,index);
    array_remove(p->child_live,index);

    lock_acquire(PID_lock);
    array_add(PID_available,(void *)pid,NULL);
    lock_release(PID_lock);

    lock_release(p->proc_lock);
  }



#endif

  if (options != 0) {
    return(EINVAL);
  }

  /* for now, just pretend the exitstatus is 0 */
  exitstatus = exitstatus;
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}


int
sys_fork(struct trapframe *tf,
         pid_t *retval)
{
#if OPT_A2

  if (PID_lock == NULL) {
    PID_lock = lock_create("PID_lock");
    if (PID_lock == NULL) {
      *retval = -1;
      return ENOMEM;
    }
  }

  // Create an array of PIDs that are available so far for later reuse
  if (PID_available == NULL) {
    PID_available = array_create();
    if (PID_available == NULL) {
      *retval = -1;
      return ENOMEM;
    }
  }

  struct proc *p = curproc;
  // Create a new process
  struct proc *child = proc_create_runprogram(p->p_name);
  if (child == NULL) {
    if (PID_lock != NULL) {
      lock_destroy(PID_lock);
    }
    *retval = -1;
    return ENOMEM;
  }

  // Create and copy address_space to child address_space
  struct addrspace *child_as;
  int error = as_copy(p->p_addrspace,&child_as);
  if (error != 0) {
    proc_destroy(child);
    *retval = -1;
    return error;
  }
  
  as_activate();

  // Associate the address_space created with child process
  child->p_addrspace = child_as;

  // Assign PID to child process and create the parent/child relationship
  lock_acquire(PID_lock);

  if (array_num(PID_available) <= 0) {
    if (PID_COUNTER > PID_MAX) {  // out of range
      proc_destroy(child);
      if (PID_lock != NULL) {
        lock_destroy(PID_lock);
      }
      lock_release(PID_lock);
      *retval = -1;
      return ENPROC; // There are already too many processes on the system
    }
    // Assign PID to child and increment the PID_COUNTER
    child->pid = PID_COUNTER;
    // kprintf("NOTTTTTTTTTTTREUSING!!!!!!!!!!!!!!\n");
    ++PID_COUNTER;
  } else {
    // Reuse a resuable PID
    child->pid = (pid_t)array_get(PID_available,0);
    // kprintf("REUSING!!!!!!!!!!!!!!\n");
    array_remove(PID_available,0);
  }

  child->parent = p;

  // Create trapframe for child process' thread
  //    need to copy to heap just in case curproc goes back to 
  //    userspace before child process going back to userspace
  struct trapframe *child_tf = kmalloc(sizeof(struct trapframe));
  *child_tf = *tf;

  lock_release(PID_lock);

  lock_acquire(p->proc_lock);
  // Add child into proc->children array
  array_add(p->children_pid,(void *)child->pid,NULL);
  array_add(p->exitcode,(void *)0,NULL);
  array_add(p->child_live,(void *)1,NULL);
  array_add(p->children,(void *)child,NULL);
  lock_release(p->proc_lock);

  // Create thread for child process
  error = thread_fork(curthread->t_name,child,&enter_forked_process,child_tf,0);
  if (error != 0) {
    // proc_destroy calls as_destroy if as is not NULL
    //  so don;t have to call as_destroy here
    kfree(child_tf);
    proc_destroy(child);
    if (PID_lock != NULL) {
      lock_destroy(PID_lock);
    }
    *retval = -1;
    return error;
  }

  // Set parent return value to child's pid
  *retval = child->pid;

#endif
  return 0;

}

int sys_execv(const_userptr_t progname, const_userptr_t *args, int *retval) {

#if OPT_A2

  int error;
  unsigned long nargs = 0;

  while (args[nargs] != NULL) {
    ++nargs;
  }

/*
  // This Commented part is copying arguments from user_stack to kernel_stack instead of using heap
  unsigned int total = 0;
  unsigned int bytes[nargs];

  // Calculate the total length of all arguments
  for (unsigned long i = 0; i < nargs; ++i) {
    // Align to 8*n bytes
    bytes[i] = strlen((char *)args[i])+1;
    total = total + bytes[i];
  }

  char all_together[total];
  unsigned int cur = 0;
  for (unsigned long i = 0; i < nargs; ++i) {
    for (unsigned long j = 0; j < strlen((char *)args[i])+1; ++j) {
      copyin(args[i]+j,&all_together[cur],1);
      ++cur;
    }
  }

  char *arguments[nargs];
  cur = 0;
  // Copy program arguments to kernel stack
  for (unsigned long i = 0; i < nargs; ++i) {
    arguments[i] = all_together+cur;
    cur = cur + bytes[i];
  }
*/


  char *arguments[nargs];
  // Copy program arguments to kernel (heap)
  for (unsigned long i = 0; i < nargs; ++i) {
    arguments[i] = kmalloc(sizeof(char)*(strlen((char *)args[i])+1));
    size_t GOT;
    error = copyinstr(args[i],arguments[i],ARG_MAX,&GOT);
    if (error != 0) {
      for (unsigned long i = 0; i < nargs; ++i) {
        kfree(arguments[i]);
      }
      *retval = -1;
      return error;
    }
  }


  size_t GOT;
  int len_path = strlen((char *)progname)+1; 
  char progpath[len_path];
  // Copy program path to kernel stack
  error = copyinstr(progname,progpath,PATH_MAX,&GOT);
  if (error != 0) {
      for (unsigned long i = 0; i < nargs; ++i) {
        kfree(arguments[i]);
      }
    *retval = -1;
    return error;
  }

  // Following are almost identical from runprogram.c
  struct addrspace *as;
  struct vnode *v;
  vaddr_t entrypoint, stackptr;

  char *fname_temp = kstrdup(progpath);
  /* Open the file. */
  error = vfs_open(fname_temp, O_RDONLY, 0, &v);
  if (error) {
      for (unsigned long i = 0; i < nargs; ++i) {
        kfree(arguments[i]);
      }
    kfree(fname_temp);
    return error;
  }
  kfree(fname_temp);

  /* Create a new address space. */
  as = as_create();
  if (as ==NULL) {
      for (unsigned long i = 0; i < nargs; ++i) {
        kfree(arguments[i]);
      }
    vfs_close(v);
    return ENOMEM;
  }

  /* Switch to it and activate it. */
  struct addrspace *old = curproc_setas(as);
  as_activate();

  /* Load the executable. */
  error = load_elf(v, &entrypoint);
  if (error) {
      for (unsigned long i = 0; i < nargs; ++i) {
        kfree(arguments[i]);
      }
    /* p_addrspace will go away when curproc is destroyed */
    vfs_close(v);
    return error;
  }

  /* Done with the file now. */
  vfs_close(v);

  // Destroy the old addrspace
  as_destroy(old);

  /* Define the user stack in the address space */
  error = as_define_stack(as, &stackptr, arguments, nargs);
  if (error) {
      for (unsigned long i = 0; i < nargs; ++i) {
        kfree(arguments[i]);
      }
    /* p_addrspace will go away when curproc is destroyed */
    return error;
  }

  // Free each arguments from heap
  for (unsigned long i = 0; i < nargs; ++i) {
    kfree(arguments[i]);
  }

  /* Warp to user mode. */
  enter_new_process((int)nargs /*argc*/, (userptr_t)stackptr /*userspace addr of argv*/,
        stackptr, entrypoint);

  /* enter_new_process does not return. */
  panic("enter_new_process returned\n");

  // Shouldn't come here
  for (unsigned long i = 0; i < nargs; ++i) {
    kfree(arguments[i]);
  }

  return EINVAL;

#endif
  *retval = ENODEV;
  return -1; 
}
