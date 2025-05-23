/****************************************************************************
 * sched/semaphore/sem_tickwait.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_tickwait
 *
 * Description:
 *   This function is a lighter weight version of sem_timedwait().  It is
 *   non-standard and intended only for use within the RTOS.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to nxsem_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   -ETIMEDOUT is returned on the timeout condition.
 *
 ****************************************************************************/

int nxsem_tickwait(FAR sem_t *sem, uint32_t delay)
{
  FAR struct tcb_s *rtcb;
  irqstate_t flags;
  int ret;

  /* We will disable interrupts until we have completed the semaphore
   * wait.  We need to do this (as opposed to just disabling pre-emption)
   * because there could be interrupt handlers that are asynchronously
   * posting semaphores and to prevent race conditions with watchdog
   * timeout.  This is not too bad because interrupts will be re-
   * enabled while we are blocked waiting for the semaphore.
   */

  flags = enter_critical_section();

  /* Try to take the semaphore without waiting. */

  ret = nxsem_trywait(sem);
  if (ret == OK)
    {
      /* We got it! */

      goto out;
    }

  /* We will have to wait for the semaphore.  Make sure that we were provided
   * with a valid timeout.
   */

  if (delay == 0)
    {
      /* Timed out already before waiting */

      ret = -ETIMEDOUT;
      goto out;
    }

  rtcb = this_task();

  /* Start the watchdog with interrupts still disabled */

  wd_start(&rtcb->waitdog, delay, nxsem_timeout, (uintptr_t)rtcb);

  /* Now perform the blocking wait */

  ret = nxsem_wait(sem);

  /* Stop the watchdog timer */

  wd_cancel(&rtcb->waitdog);

  /* We can now restore interrupts */

out:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: nxsem_tickwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_tickwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to sem_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure:
 *
 *     -ETIMEDOUT is returned on the timeout condition.
 *     -ECANCELED may be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int nxsem_tickwait_uninterruptible(FAR sem_t *sem, uint32_t delay)
{
  clock_t end = clock_delay2abstick(delay);
  int ret;

  for (; ; )
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_tickwait(sem, delay);
      if (ret != -EINTR)
        {
          break;
        }

      delay = end - clock_systime_ticks();
      if ((int32_t)delay < 0)
        {
          delay = 0;
        }
    }

  return ret;
}
