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
// static struct semaphore *intersectionSem;
static struct lock *CheckLock;

// Static struct condition variable
static struct cv *cv_east;
static struct cv *cv_south;
static struct cv *cv_north;
static struct cv *cv_west;


// Global variables to keep track of the # of vehicles in each possible route
// Each global variable indicates a distinct Route
static volatile int cur_east = 0;
static volatile int cur_west = 0;
static volatile int cur_north = 0;
static volatile int cur_south = 0;

static volatile int MAX = 0;
static volatile int WEST = 0;
static volatile int NORTH = 0;
static volatile int SOUTH = 0;
static volatile int EAST = 0;

static volatile bool is_first_entry = true;
static volatile int num_of_vehicles = 0;
static volatile Direction current;

bool wchan_isempty(struct wchan *wc);

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
/*
  intersectionSem = sem_create("intersectionSem",1);
  if (intersectionSem == NULL) {
    panic("could not create intersection semaphore");
  }
*/
  CheckLock = lock_create("CheckLock");
  if (CheckLock == NULL) {
    panic("could not create CheckLock");
  }

  // Create 12 condition variables for each route 
  // The condition for each route is to check whether the sum of 
  //    all cars on the "prohibited" route equal 0
  //    If the sum == 0, then the vehicle can enter intersection
  //    Otherwise, the vehicle has to wait until the sum == 0
  cv_north = cv_create("NORTH");
  if (cv_north == NULL) {
    panic("could not create Condition variable SW");
  }
  
  cv_south = cv_create("SOUTH");
  if (cv_south == NULL) {
    panic("could not create Condition variable SN");
  }
  
  cv_east = cv_create("EAST");
  if (cv_east == NULL) {
    panic("could not create Condition variable SE");
  }

  cv_west = cv_create("WEST");
  if (cv_west == NULL) {
    panic("could not create Condition variable WN");
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
/*  KASSERT(intersectionSem != NULL);
  sem_destroy(intersectionSem);
*/

  KASSERT(CheckLock != NULL);
  lock_destroy(CheckLock);

  // Destroy all 12 condition variables
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
//  KASSERT(intersectionSem != NULL);
  KASSERT(CheckLock != NULL);

  // Lock the "CHECK" critical section to ensure that
  //    you are the only one who is checking the intersection right now
//  P(intersectionSem);
  lock_acquire(CheckLock);

  if (is_first_entry) {
    current = origin;
    is_first_entry = false;
  }

  // Divide into 12 distinct routes
  //    check the condition to enter the intersection to 
  //    determine whether it is able to enter or wait
  switch (origin) {
    case north: 
                while (current != origin) { // vehicles from north can enter
                  ++NORTH;
                  cv_wait(cv_north,CheckLock);
                }
      break;
    case east: 
                while (current != origin) { // vehicles from east can enter
                  ++EAST;
                  cv_wait(cv_east,CheckLock);
                }   
      break;
    case south:    
                while (current != origin) { // vehicles from south can enter                 
                  ++SOUTH;
                  cv_wait(cv_south,CheckLock);
                }          
      break;
    case west: 
                while (current != origin) { // vehicles from west can enter                 
                  ++WEST;
                  cv_wait(cv_west,CheckLock);
                }
      break; 
  }

  ++num_of_vehicles;
  // Unlock the "CHECK" semaphore after finishing checking the intersection
  // V(intersectionSem);
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
  // KASSERT(intersectionSem != NULL);
  KASSERT(CheckLock != NULL);

  // Lock when leave the intersection
  // P(intersectionSem);
  lock_acquire(CheckLock);


  // Decrement the corresponding Route global variable
  --num_of_vehicles;

  if (num_of_vehicles == 0) {
      if (wchan_isempty(cv_north->cv_wchan) &&
          wchan_isempty(cv_south->cv_wchan) && 
          wchan_isempty(cv_east->cv_wchan) &&
          wchan_isempty(cv_west->cv_wchan)) {
        is_first_entry = true;
      } else {

        if (WEST >= SOUTH && WEST >= NORTH && WEST >= EAST) {
          MAX = WEST;
        } else if (EAST >= SOUTH && EAST >= NORTH && EAST >= WEST) {
          MAX = EAST;
        } else if (NORTH >= EAST && NORTH >= SOUTH && NORTH >= WEST) {
          MAX = NORTH;
        } else if (SOUTH >= EAST && SOUTH >= NORTH && SOUTH >= WEST) {
          MAX = SOUTH;
        }

        if (MAX == WEST) {
          WEST = 0;
          current = west;
          cv_broadcast(cv_west,CheckLock);
        } else if (MAX == EAST) {
          EAST = 0;
          current = east;
          cv_broadcast(cv_east,CheckLock);
        } else if (MAX == SOUTH) {
          SOUTH = 0;
          current = south;
          cv_broadcast(cv_south,CheckLock);
        } else { // MAX == NORTH
          NORTH = 0;
          current = north;
          cv_broadcast(cv_north,CheckLock);
        }
      }
  }

  // Unlock when the last step before leaving the intersection
  // V(intersectionSem);
  lock_release(CheckLock);

  return;
}


