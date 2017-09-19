#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */

// Lock to check if condition met before entering the intersection and update counter after leaving
static struct lock *CheckLock;

// Static struct condition variable for each different origin
static struct cv *cv_east;
static struct cv *cv_south;
static struct cv *cv_north;
static struct cv *cv_west;

// current origin in the intersection (only allow on origin at a time)
static volatile Direction current;

// True when allow next coming vehicle to determine the current origin
// False when the current origin is determined already
static volatile bool cv_intersection_empty = true;

// Global variables to keep track of the # of vehicles currently in the intersection
static volatile int num_of_vehicles = 0;

// To accumulate the # of vehicles that has passed the intersection so far (like a clock)
static volatile int COUNTER = 0;

// To keep track of the counter (time) of first waiting vehicle in the respective origin
// -1 indicates no vehicles are waiting 
static volatile int WEST = -1;
static volatile int NORTH = -1;
static volatile int SOUTH = -1;
static volatile int EAST = -1;
 
// The minimum of WEST, EAST, NORTH, SOUTH if not -1;
static volatile int MIN = 0;


// Helper to determine the minimum of four integers >=0
//  -1 will be omitted, because it indicates no vehicles are waiting
static int min_of_four(int a, int b, int c, int d) {

  int min = 0;
  if (a >= min) { // just mean a != 0
    min = a;
  } else if (b >= min) {
    min = b;
  } else if (c >= min) {
    min = c;
  } else { // a, b, c, d as least one != -1, otherwise this function will not be called
    min = d;
  }

  // determine the min integer (exclude -1)
  if (a < min && a != -1) {
    min = a;
  }
  if (b < min && b != -1) {
    min = b;
  }
  if (c < min && c != -1) {
    min = c;
  }
  if (d < min && d != -1) {
    min = d;
  }

  return min;
}


/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  // Create the lock
  CheckLock = lock_create("CheckLock");
  if (CheckLock == NULL) {
    panic("could not create CheckLock");
  }

  // Create 4 condition variables for each origin
  cv_north = cv_create("NORTH");
  if (cv_north == NULL) {
    panic("could not create Condition variable NORTH");
  }
  
  cv_south = cv_create("SOUTH");
  if (cv_south == NULL) {
    panic("could not create Condition variable SOUTH");
  }
  
  cv_east = cv_create("EAST");
  if (cv_east == NULL) {
    panic("could not create Condition variable EAST");
  }

  cv_west = cv_create("WEST");
  if (cv_west == NULL) {
    panic("could not create Condition variable WEST");
  }

  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */

  KASSERT(CheckLock != NULL);
  // Destroy the lock
  lock_destroy(CheckLock);

  // Destroy all 4 condition variables
  //    Dont' have to check whether CV == NULL here
  //    CV_destory has KASSERT(CV != NULL)
  cv_destroy(cv_north);
  cv_destroy(cv_west);
  cv_destroy(cv_south);
  cv_destroy(cv_east);

  return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  //  (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  KASSERT(CheckLock != NULL);
  KASSERT(cv_east != NULL);
  KASSERT(cv_west != NULL);
  KASSERT(cv_north != NULL);
  KASSERT(cv_south != NULL);

  // Lock the critical section to ensure that
  //    you are the only one who is checking the intersection right now
  lock_acquire(CheckLock);

  // To check if the current origin has been decided already
  if (cv_intersection_empty) {
    current = origin;
    cv_intersection_empty = false;
  }

  while (current != origin) {  // check if the vehicle can enter, only allow 
                               // vehicles of same origin enter
             
    // Divide into 4 distinct cases based on the origin of the vehicles
    //    check the condition (only allow same origin) to enter the intersection to 
    //    determine whether it is able to enter or wait
    switch (origin) {
      case north: 
                    if (-1 == NORTH) { // if empty, then this is the first waiting
                                              //   vehicle in this CV, modify the corresponding
                                              //   entry counter to current counter
                      NORTH = COUNTER;
                    }
                    cv_wait(cv_north,CheckLock); // cannot enter, waiting on the corresponding CV
        break;
      case east: 
                    if (-1 == EAST) {
                      EAST = COUNTER;
                    }
                    cv_wait(cv_east,CheckLock);              
        break;
      case south:                     
                    if (-1 == SOUTH) {
                      SOUTH = COUNTER;
                    }
                    cv_wait(cv_south,CheckLock);                              
        break;
      case west:                
                    if (-1 == WEST) {
                      WEST = COUNTER;
                    }
                    cv_wait(cv_west,CheckLock);
        break;
    }
  }

  // increment the number of vehicles currently in the intersection
  ++num_of_vehicles;
  // increment the counter of vehicles that has entered so far
  ++COUNTER;

  // Unlock the lock after finishing checking the intersection
  lock_release(CheckLock);
  return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  KASSERT(CheckLock != NULL);
  KASSERT(cv_east != NULL);
  KASSERT(cv_west != NULL);
  KASSERT(cv_north != NULL);
  KASSERT(cv_south != NULL);

  // Get lock when leave the intersection
  lock_acquire(CheckLock);


  // Decrement the number of vehicles currently in the intersection
  --num_of_vehicles;
  
  // intersection currently empty
  if (num_of_vehicles == 0) {

    // no vehicles are waiting from any origin CVs
    if ((-1 == WEST) && (-1 == EAST) && (-1 == SOUTH) && (-1 == NORTH)) {
      cv_intersection_empty = true; // allow next vehicle to determine the current origin
    } else {

      // determine the minimum of each entry counter to decide which origin has been 
      //    waited for longest period of time (Note: MIN means arrives ealiest waiting 
      //    but not entered, so waiting longest time), then broadcast the MIN one
      MIN = min_of_four(EAST,WEST,SOUTH,NORTH);

      if (MIN == WEST) {
        current = west; // update the current origin, which enables that the vehicles
                        //     being waked up will not be waiting again
                        //     (kind of immediately enter intersection)
        WEST = -1;
        cv_broadcast(cv_west,CheckLock); // wake up all vehicles that are on the corresponding CV
      } else if (MIN == EAST) {
        current = east;
        EAST = -1;
        cv_broadcast(cv_east,CheckLock);
      } else if (MIN == SOUTH) {
        current = south;
        SOUTH = -1;
        cv_broadcast(cv_south,CheckLock);
      } else { // (MIN == NORTH) 
        current = north;
        NORTH = -1;
        cv_broadcast(cv_north,CheckLock);
      }

    }
  }

  // Unlock when the last step before leaving the intersection
  lock_release(CheckLock);

  return;
}


