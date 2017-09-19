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
static struct cv *cv_NE;
static struct cv *cv_NW;
static struct cv *cv_NS;

static struct cv *cv_EW;
static struct cv *cv_ES;
static struct cv *cv_EN;

static struct cv *cv_WS;
static struct cv *cv_WE;
static struct cv *cv_WN;

static struct cv *cv_SE;
static struct cv *cv_SW;
static struct cv *cv_SN;


// current origin in the intersection (only allow on origin at a time)
static volatile Direction current_origin, current_destination;

// True when allow next coming vehicle to determine the current origin
// False when the current origin is determined already
static volatile bool cv_intersection_empty = true;

// Global variables to keep track of the # of vehicles currently in the intersection
static volatile int num_of_vehicles = 0;

// To accumulate the # of vehicles that has passed the intersection so far (like a clock)
static volatile int COUNTER = 0;

// To keep track of the counter (time) of first waiting vehicle in the respective origin
// -1 indicates no vehicles are waiting 
static volatile int WN = -1;
static volatile int WE = -1;
static volatile int WS = -1;
static volatile int ES = -1;
static volatile int EW = -1;
static volatile int EN = -1;
static volatile int NW = -1;
static volatile int NS = -1;
static volatile int NE = -1;
static volatile int SE = -1;
static volatile int SN = -1;
static volatile int SW = -1;
 
// The minimum of WEST, EAST, NORTH, SOUTH if not -1;
static volatile int MIN = 0;


// Helper to determine the minimum of four integers >=0
//  -1 will be omitted, because it indicates no vehicles are waiting
static int min_of_12(int a, int b, int c, int d, int e, int f,
                     int g, int h, int i, int j, int k, int l) {

  int min = 0;
  if (a >= min) { // just mean a != 0
    min = a;
  } else if (b >= min) {
    min = b;
  } else if (c >= min) {
    min = c;
  } else if (d >= min) {
    min = d;
  } else if (e >= min) {
    min = e;
  } else if (f >= min) {
    min = f;
  } else if (g >= min) {
    min = g;
  } else if (h >= min) {
    min = h;
  } else if (i >= min) {
    min = i;
  } else if (j >= min) {
    min = j;
  } else if (k >= min) {
    min = k;
  } else { // a, b, c, d as least one != -1, otherwise this function will not be called
    min = l;
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
  if (e < min && e != -1) {
    min = e;
  }
  if (f < min && f != -1) {
    min = f;
  }
  if (g < min && g != -1) {
    min = g;
  }
  if (h < min && h != -1) {
    min = h;
  }
  if (i < min && i != -1) {
    min = i;
  }
  if (j < min && j != -1) {
    min = j;
  }
  if (k < min && k != -1) {
    min = k;
  }
  if (l < min && l != -1) {
    min = l;
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
  cv_NE = cv_create("NORTH");
  cv_NW = cv_create("SOUTH");
  cv_NS = cv_create("EAST");
  cv_WE = cv_create("WEST");
  cv_WN = cv_create("NORTH");
  cv_WS = cv_create("SOUTH");
  cv_ES = cv_create("EAST");
  cv_EW = cv_create("WEST");
  cv_EN = cv_create("NORTH");
  cv_SE = cv_create("SOUTH");
  cv_SW = cv_create("EAST");
  cv_SN = cv_create("WEST");

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
  cv_destroy(cv_NE);
  cv_destroy(cv_NS);
  cv_destroy(cv_NW);
  cv_destroy(cv_EN);
  cv_destroy(cv_EW);
  cv_destroy(cv_ES);
  cv_destroy(cv_WS);
  cv_destroy(cv_WN);
  cv_destroy(cv_WE);
  cv_destroy(cv_SE);
  cv_destroy(cv_SN);
  cv_destroy(cv_SW);

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
  //  (void)destination; /* avoid compiler complaint about unused parameter */
  KASSERT(CheckLock != NULL);

  // Lock the critical section to ensure that
  //    you are the only one who is checking the intersection right now
  lock_acquire(CheckLock);

  // To check if the current origin has been decided already
  if (cv_intersection_empty) {
    current_origin = origin;
    current_destination = destination;
    cv_intersection_empty = false;
  }

  while ((current_origin != origin) || (current_destination != destination)) {  // check if the vehicle can enter, only allow 
                               // vehicles of same origin enter
             
    // Divide into 4 distinct cases based on the origin of the vehicles
    //    check the condition (only allow same origin) to enter the intersection to 
    //    determine whether it is able to enter or wait
    switch (origin) {
      case north: 
                if (destination == east) {
                    if (-1 == NE) {
                      NE = COUNTER;
                    }
                    cv_wait(cv_NE,CheckLock); 
                } else if (destination == south) {
                    if (-1 == NS) {
                      NS = COUNTER;
                    }
                    cv_wait(cv_NS,CheckLock); 
                } else if (destination == west) {
                    if (-1 == NW) {
                      NW = COUNTER;
                    }
                    cv_wait(cv_NW,CheckLock); 
                }
        break;
      case east: 
                if (destination == west) {
                    if (-1 == EW) {
                      EW = COUNTER;
                    }
                    cv_wait(cv_EW,CheckLock); 
                } else if (destination == south) {
                    if (-1 == ES) {
                      ES = COUNTER;
                    }
                    cv_wait(cv_ES,CheckLock); 
                } else if (destination == north) {
                    if (-1 == EN) {
                      EN = COUNTER;
                    }
                    cv_wait(cv_EN,CheckLock); 
                }             
        break;
      case south:                     
                if (destination == east) {
                    if (-1 == SE) {
                      SE = COUNTER;
                    }
                    cv_wait(cv_SE,CheckLock); 
                } else if (destination == north) {
                    if (-1 == SN) {
                      SN = COUNTER;
                    }
                    cv_wait(cv_SN,CheckLock); 
                } else if (destination == west) {
                    if (-1 == SW) {
                      SW = COUNTER;
                    }
                    cv_wait(cv_SW,CheckLock); 
                }                            
        break;
      case west:                
                if (destination == east) {
                    if (-1 == WE) {
                      WE = COUNTER;
                    }
                    cv_wait(cv_WE,CheckLock); 
                } else if (destination == south) {
                    if (-1 == WS) {
                      WS = COUNTER;
                    }
                    cv_wait(cv_WS,CheckLock); 
                } else if (destination == north) {
                    if (-1 == WN) {
                      WN = COUNTER;
                    }
                    cv_wait(cv_WN,CheckLock); 
                }
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

  // Get lock when leave the intersection
  lock_acquire(CheckLock);


  // Decrement the number of vehicles currently in the intersection
  --num_of_vehicles;
  
  // intersection currently empty
  if (num_of_vehicles == 0) {

    // no vehicles are waiting from any origin CVs
    if ((-1 == NW) && (-1 == NE) && (-1 == NS) &&
        (-1 == SW) && (-1 == SE) && (-1 == SN) &&
        (-1 == WN) && (-1 == WE) && (-1 == WS) &&
        (-1 == ES) && (-1 == EN) && (-1 == EW)) {
      cv_intersection_empty = true; // allow next vehicle to determine the current origin
    } else {

      // determine the minimum of each entry counter to decide which origin has been 
      //    waited for longest period of time (Note: MIN means arrives ealiest waiting 
      //    but not entered, so waiting longest time), then broadcast the MIN one
      MIN = min_of_12(NE,NW,NS,EW,ES,EN,WS,WE,WN,SW,SE,SN);

      if (MIN == WN) {
        current_origin = west;
        current_destination = north;
        WN = -1;
        cv_broadcast(cv_WN,CheckLock);

      } else if (MIN == WE) {
        current_origin = west;
        current_destination = east;
        WE = -1;
        cv_broadcast(cv_WE,CheckLock);

      } else if (MIN == WS) {
        current_origin = west;
        current_destination = south;
        WS = -1;
        cv_broadcast(cv_WS,CheckLock);

      } else if (MIN == EW) {
        current_origin = east;
        current_destination = west;
        EW = -1;
        cv_broadcast(cv_EW,CheckLock);

      } else if (MIN == EN) {
        current_origin = east;
        current_destination = north;
        EN = -1;
        cv_broadcast(cv_EN,CheckLock);

      } else if (MIN == ES) {
        current_origin = east;
        current_destination = south;
        ES = -1;
        cv_broadcast(cv_ES,CheckLock);

      } else if (MIN == NS) {
        current_origin = north;
        current_destination = south;
        NS = -1;
        cv_broadcast(cv_NS,CheckLock);

      } else if (MIN == NE) {
        current_origin = north;
        current_destination = east;
        NE = -1;
        cv_broadcast(cv_NE,CheckLock);

      } else if (MIN == NW) {
        current_origin = north;
        current_destination = west;
        NW = -1;
        cv_broadcast(cv_NW,CheckLock);

      } else if (MIN == SW) {
        current_origin = south;
        current_destination = west;
        SW = -1;
        cv_broadcast(cv_SW,CheckLock);

      } else if (MIN == SN) {
        current_origin = south;
        current_destination = north;
        SN = -1;
        cv_broadcast(cv_SN,CheckLock);

      } else if (MIN == SE) {
        current_origin = south;
        current_destination = east;
        SE = -1;
        cv_broadcast(cv_SE,CheckLock);
      }





    }
  }

  // Unlock when the last step before leaving the intersection
  lock_release(CheckLock);

  return;
}


