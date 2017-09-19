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
static struct cv *cv_SW;
static struct cv *cv_SN;
static struct cv *cv_SE;

static struct cv *cv_WN;
static struct cv *cv_WE;
static struct cv *cv_WS;
  
static struct cv *cv_NE;
static struct cv *cv_NS;
static struct cv *cv_NW;

static struct cv *cv_ES;
static struct cv *cv_EW;
static struct cv *cv_EN;

// Global variables to keep track of the # of vehicles in each possible route
// Each global variable indicates a distinct Route
static volatile int SW = 0;
static volatile int SN = 0;
static volatile int SE = 0;

static volatile int WN = 0;
static volatile int WE = 0;
static volatile int WS = 0;

static volatile int NE = 0;
static volatile int NS = 0;
static volatile int NW = 0;

static volatile int ES = 0;
static volatile int EW = 0;
static volatile int EN = 0;

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
  cv_SW = cv_create("0_WE_WN_NS_NW_NE_EW_ES");
  if (cv_SW == NULL) {
    panic("could not create Condition variable SW");
  }
  
  cv_SN = cv_create("0_WE_WN_NE_EW_EN_ES");
  if (cv_SN == NULL) {
    panic("could not create Condition variable SN");
  }
  
  cv_SE = cv_create("0_WE_NE");
  if (cv_SE == NULL) {
    panic("could not create Condition variable SE");
  }


  cv_WN = cv_create("0_NS_NE_EW_EN_ES_SN_SW");
  if (cv_WN == NULL) {
    panic("could not create Condition variable WN");
  }

  cv_WE = cv_create("0_NS_NE_ES_SN_SE_SW");
  if (cv_WE == NULL) {
    panic("could not create Condition variable WE");
  }

  cv_WS = cv_create("0_NS_ES");
  if (cv_WS == NULL) {
    panic("could not create Condition variable WS");
  }


  cv_NE = cv_create("0_EW_ES_SN_SE_SW_WE_WN");
  if (cv_NE == NULL) {
    panic("could not create Condition variable NE");
  }

  cv_NS = cv_create("0_EW_ES_SW_WN_WE_WS");
  if (cv_NS == NULL) {
    panic("could not create Condition variable NS");
  }

  cv_NW = cv_create("0_EW_SW");
  if (cv_NW == NULL) {
    panic("could not create Condition variable NW");
  }


  cv_ES = cv_create("0_SN_SW_WE_WS_WN_NS_NE");
  if (cv_ES == NULL) {
    panic("could not create Condition variable ES");
  }

  cv_EW = cv_create("0_SN_SW_WN_NS_NW_NE");
  if (cv_EW == NULL) {
    panic("could not create Condition variable EW");
  }

  cv_EN = cv_create("0_SN_WN");
  if (cv_EN == NULL) {
    panic("could not create Condition variable EN");
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
  cv_destroy(cv_SW);
  cv_destroy(cv_SN);
  cv_destroy(cv_SE);

  cv_destroy(cv_WN);
  cv_destroy(cv_WE);
  cv_destroy(cv_WS);

  cv_destroy(cv_NE);
  cv_destroy(cv_NS);
  cv_destroy(cv_NW);

  cv_destroy(cv_ES);
  cv_destroy(cv_EW);
  cv_destroy(cv_EN);

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
//  KASSERT(intersectionSem != NULL);
  KASSERT(CheckLock != NULL);

  // Lock the "CHECK" critical section to ensure that
  //    you are the only one who is checking the intersection right now
//  P(intersectionSem);
  lock_acquire(CheckLock);



  // Divide into 12 distinct routes
  //    check the condition to enter the intersection to 
  //    determine whether it is able to enter or wait
  switch (origin) {

    case north:  
                if (destination == east) {
                  while (0 != (EW + ES + SN + SE + SW + WE + WN)) { // safe to enter the intersection
                    cv_wait(cv_NE,CheckLock); // not safe to enter the intersection, wait on CV
                  }
                  KASSERT(NE >= 0); // make sure the # of vehicles in the route at least 0
                  ++NE; // increment the # of vehicles in that route
                } else if (destination == south) {
                  while (0 != (EW + ES + SW + WN + WE + WS)) {
                    cv_wait(cv_NS,CheckLock);
                  }
                  KASSERT(NS >= 0);
                  ++NS;
                } else if (destination == west) {
                  while (0 != (EW + SW)) {
                    cv_wait(cv_NW,CheckLock);
                  }
                  KASSERT(NW >= 0);
                  ++NW;
                } else {
                  panic("Trying same origin and same destination");
                } 
      break;
    case east: 
                if (destination == south) {
                  while (0 != (SN + SW + WE + WS + WN + NS + NE)) {
                    cv_wait(cv_ES,CheckLock);
                  }
                  KASSERT(ES >= 0);
                  ++ES;
                } else if (destination == west) {
                  while (0 != (SN + SW + WN + NS + NW + NE)) {
                    cv_wait(cv_EW,CheckLock);
                  }
                  KASSERT(EW >= 0);
                  ++EW;
                } else if (destination == north) {
                  while (0 != (SN + WN)) {
                    cv_wait(cv_EN,CheckLock);
                  }
                  KASSERT(EN >= 0);
                  ++EN;
                } else {
                  panic("Trying same origin and same destination");
                }           
      break;
    case south:    
                if (destination == west) {
                  while (0 != (WE + WN + NS + NW + NE + EW + ES)) {
                    cv_wait(cv_SW,CheckLock);
                  }
                  KASSERT(SW >= 0);
                  ++SW;                  
                } else if (destination == north) {
                  while (0 != (WE + WN + NE + EW + EN + ES)) {
                    cv_wait(cv_SN,CheckLock);
                  }
                  KASSERT(SN >= 0);
                  ++SN;
                } else if (destination == east) {
                  while (0 != (WE + NE)) {
                    cv_wait(cv_SE,CheckLock);
                  }
                  KASSERT(SE >= 0);
                  ++SE;                    
                } else {
                  panic("Trying same origin and same destination");
                }
      break;
    case west: 
                if (destination == north) {
                  while (0 != (NS + NE + EW + EN + ES + SN + SW)) {
                    cv_wait(cv_WN,CheckLock);
                  }
                  KASSERT(WN >= 0);
                  ++WN;                   
                } else if (destination == east) {
                  while (0 != (NS + NE + ES + SN + SE + SW)) {
                    cv_wait(cv_WE,CheckLock);
                  }
                  KASSERT(WE >= 0);
                  ++WE;                  
                } else if (destination == south) {
                  while (0 != (NS + ES)) {
                    cv_wait(cv_WS,CheckLock);
                  }
                  KASSERT(WS >= 0);
                  ++WS;                    
                } else {
                  panic("Trying same origin and same destination");
                }
      break; 
  }

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
  // (void)origin;  /* avoid compiler complaint about unused parameter */
  // (void)destination; /* avoid compiler complaint about unused parameter */
  // KASSERT(intersectionSem != NULL);
  KASSERT(CheckLock != NULL);

  // Lock when leave the intersection
  // P(intersectionSem);
  lock_acquire(CheckLock);

// kprintf("%d cars in the intersection\n",NE+NW+NS+WE+WN+WS+SW+SN+SE+ES+EW+EN);

  // Decrement the corresponding Route global variable
  switch (origin) {
    case north:
                if (destination == east) {
                  KASSERT(NE > 0); // make sure the # of vehicles in a route at least 1
                  --NE; // decrement the # of vehicles in that route
                } else if (destination == south) {
                  KASSERT(NS > 0);
                  --NS;
                } else if (destination == west) {
                  KASSERT(NW > 0);
                  --NW;
                } else {
                  panic("Trying same origin and same destination");
                }
      break;
    case east:
                if (destination == south) {  
                  KASSERT(ES > 0);
                  --ES;
                } else if (destination == west) {
                  KASSERT(EW > 0);
                  --EW;
                } else if (destination == north) {
                  KASSERT(EN > 0);
                  --EN;
                } else {
                  panic("Trying same origin and same destination");
                }
      break;
    case south:
                if (destination == west) {
                  KASSERT(SW > 0);
                  --SW;
                } else if (destination == north) {
                  KASSERT(SN > 0);
                  --SN;
                } else if (destination == east) {
                  KASSERT(SE > 0);
                  --SE;                 
                } else {
                  panic("Trying same origin and same destination");
                }
      break;
    case west:
                if (destination == north) {
                    KASSERT(WN > 0);
                    --WN;
                } else if (destination == east) {
                    KASSERT(WE > 0);
                    --WE;
                } else if (destination == south) {
                     KASSERT(WS > 0);
                    --WS;                 
                } else {
                  panic("Trying same origin and same destination");
                }
      break;
  }

cv_broadcast(cv_EN,CheckLock);
cv_broadcast(cv_NW,CheckLock);
cv_broadcast(cv_SE,CheckLock);
cv_broadcast(cv_WS,CheckLock);
cv_broadcast(cv_ES,CheckLock);
cv_broadcast(cv_EW,CheckLock);
cv_broadcast(cv_NE,CheckLock);
cv_broadcast(cv_NS,CheckLock);
cv_broadcast(cv_SW,CheckLock);
cv_broadcast(cv_SN,CheckLock);
cv_broadcast(cv_WE,CheckLock);
cv_broadcast(cv_WN,CheckLock);

/*
  if (0 == (SN + WN)) {
    if (0 == (SW + WE + WS + NS + NE)) {
      cv_broadcast(cv_ES,CheckLock);
    }
    if (0 == (SW + NS + NW + NE)) {
      cv_broadcast(cv_EW,CheckLock);
    }
    cv_broadcast(cv_EN,CheckLock);
  }
  // After leave the intersection (decrement the respective global variable),
  //    check if the condition met (the vehicle can enter intersection), then 
  //    wake up all the vehicles in those corresponding condition variables 
  // Note: if same origin can either go straight or turn left, it also can turn right
  if (0 == (WE + NE)) { // turn right 
    if (0 == (WN + NS + NW + EW + ES)) { // turn left
      cv_broadcast(cv_SW,CheckLock);
    }
    if (0 == (WN + EW + EN + ES)) { // go straight
      cv_broadcast(cv_SN,CheckLock);
    }
    cv_broadcast(cv_SE,CheckLock);
  }

  if (0 == (NS + ES)) {
    if (0 == (NE + EW + EN + SN + SW)) {
      cv_broadcast(cv_WN,CheckLock);
    }    
    if (0 == (NE + SN + SE + SW)) {
      cv_broadcast(cv_WE,CheckLock);
    }
    cv_broadcast(cv_WS,CheckLock);
  }

  if (0 == (EW + SW)) {
    if (0 == (ES + SN + SE + WE + WN)) {
      cv_broadcast(cv_NE,CheckLock);
    }
    if (0 == (ES + WN + WE + WS)) {
      cv_broadcast(cv_NS,CheckLock);
    }
    cv_broadcast(cv_NW,CheckLock);
  }
*/
  // Unlock when the last step before leaving the intersection
  // V(intersectionSem);
  lock_release(CheckLock);

  return;
}


