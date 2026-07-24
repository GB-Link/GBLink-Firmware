#include "layers/linkLayer.h"

#include <zephyr/kernel.h>

/* Wrong-pin watchdog for classic slave modes (trade emu, GBA link relay, AW
 * slave). Cable detection is a heuristic and can pick the wrong SD pin (GBA
 * already in link mode at detect time, or cable plugged in afterwards). The
 * partner clocking SC while zero words arrive is proof of the wrong pin — SC
 * is the same pin for both cable types — so after sustained evidence the
 * watchdog flips the SD path and reconfigures, once per armed session.
 * Requiring both SC levels (a real toggle) rejects a stuck line, and any
 * received word disarms the watchdog until the next configure. */
static void wrongPinWatchdogThread(void* a, void* b, void* c)
{
    (void)a; (void)b; (void)c;
    bool scSeenLow = false;
    bool scSeenHigh = false;
    int ticks = 0;
    int clockingIntervals = 0;

    for (;;)
    {
        if (link_getSlaveMode() != CLASSIC || link_getWrongPinFlipsLeft() == 0)
        {
            scSeenLow = scSeenHigh = false;
            ticks = 0;
            clockingIntervals = 0;
            k_sleep(K_MSEC(100));
            continue;
        }

        k_sleep(K_MSEC(1));
        if (link_readPartnerPins() & 0x01) scSeenLow = true;
        else scSeenHigh = true;

        if (++ticks < 550) continue;
        ticks = 0;

        const bool scToggled = scSeenLow && scSeenHigh;
        scSeenLow = scSeenHigh = false;

        if (link_getReceivedWordCount() != 0)
        {
            clockingIntervals = 0;
            continue;
        }
        if (scToggled) clockingIntervals++;

        if (clockingIntervals >= 4)
        {
            clockingIntervals = 0;
            link_tryWrongPinRecovery();
        }
    }
}

K_THREAD_DEFINE(wrongPinWatchdog_tid, 768, wrongPinWatchdogThread,
                NULL, NULL, NULL, 12, 0, 0);
