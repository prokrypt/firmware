#include "TestUtil.h"
#include "TransmitHistory.h"
#include "UptimeClock.h"
#include <Throttle.h>
#include <unity.h>

// Reset the singleton between tests
static void resetTransmitHistory()
{
    if (transmitHistory) {
        delete transmitHistory;
        transmitHistory = nullptr;
    }
    transmitHistory = TransmitHistory::getInstance();
}

void setUp(void)
{
    resetTransmitHistory();
}

void tearDown(void)
{
    Time::useRealClock();
}

static void test_setLastSentToMesh_stores_millis()
{
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_NODEINFO_APP);

    uint32_t result = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_NODEINFO_APP);
    TEST_ASSERT_NOT_EQUAL(0, result);

    // The stored millis value should be very close to current millis()
    uint32_t diff = millis() - result;
    TEST_ASSERT_LESS_OR_EQUAL(100, diff); // Within 100ms
}

static void test_set_overwrites_previous_value()
{
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_TELEMETRY_APP);
    uint32_t first = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_TELEMETRY_APP);

    testDelay(50);

    transmitHistory->setLastSentToMesh(meshtastic_PortNum_TELEMETRY_APP);
    uint32_t second = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_TELEMETRY_APP);

    // The second value should be newer (larger millis)
    TEST_ASSERT_GREATER_THAN(first, second);
}

// --- Throttle integration ---

static void test_throttle_blocks_within_interval()
{
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_NODEINFO_APP);
    uint32_t lastMs = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_NODEINFO_APP);

    // Should be within a 10-minute interval (just set it)
    bool withinInterval = Throttle::isWithinTimespanMs(lastMs, 10 * 60 * 1000);
    TEST_ASSERT_TRUE(withinInterval);
}

static void test_throttle_allows_after_interval()
{
    // Unknown key returns 0 - throttle should NOT block
    uint32_t lastMs = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_NODEINFO_APP);
    TEST_ASSERT_EQUAL_UINT32(0, lastMs);

    // When lastMs == 0, the module check `lastMs == 0 || !isWithinTimespan` allows sending
    bool shouldSend = (lastMs == 0) || !Throttle::isWithinTimespanMs(lastMs, 10 * 60 * 1000);
    TEST_ASSERT_TRUE(shouldSend);
}

static void test_throttle_blocks_after_set_then_zero_does_not()
{
    // Set it - now throttle should block
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_TELEMETRY_APP);
    uint32_t lastMs = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_TELEMETRY_APP);
    bool shouldSend = (lastMs == 0) || !Throttle::isWithinTimespanMs(lastMs, 60 * 60 * 1000);
    TEST_ASSERT_FALSE(shouldSend); // Should be blocked (within 1hr interval)

    // Different key - should allow
    uint32_t otherMs = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_POSITION_APP);
    bool otherShouldSend = (otherMs == 0) || !Throttle::isWithinTimespanMs(otherMs, 60 * 60 * 1000);
    TEST_ASSERT_TRUE(otherShouldSend);
}

// --- Multiple keys ---

static void test_multiple_keys_stored_independently()
{
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_NODEINFO_APP);
    uint32_t nodeInfoInitial = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_NODEINFO_APP);
    testDelay(20);
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_POSITION_APP);
    uint32_t positionInitial = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_POSITION_APP);
    testDelay(20);
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_TELEMETRY_APP);

    uint32_t nodeInfo = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_NODEINFO_APP);
    uint32_t position = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_POSITION_APP);
    uint32_t telemetry = transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_TELEMETRY_APP);

    // All should be non-zero
    TEST_ASSERT_NOT_EQUAL(0, nodeInfo);
    TEST_ASSERT_NOT_EQUAL(0, position);
    TEST_ASSERT_NOT_EQUAL(0, telemetry);

    // Updating other keys should not overwrite earlier key timestamps
    TEST_ASSERT_EQUAL_UINT32(nodeInfoInitial, nodeInfo);
    TEST_ASSERT_EQUAL_UINT32(positionInitial, position);
}

// --- Singleton ---

static void test_getInstance_returns_same_instance()
{
    TransmitHistory *a = TransmitHistory::getInstance();
    TransmitHistory *b = TransmitHistory::getInstance();
    TEST_ASSERT_EQUAL_PTR(a, b);
}

static void test_getInstance_creates_global()
{
    if (transmitHistory) {
        delete transmitHistory;
        transmitHistory = nullptr;
    }
    TEST_ASSERT_NULL(transmitHistory);

    TransmitHistory::getInstance();
    TEST_ASSERT_NOT_NULL(transmitHistory);
}

// --- RAM only ---

static void test_stamps_do_not_survive_a_new_instance()
{
    // A reboot is a fresh instance; nothing is read back from flash, so every key reads as never sent.
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_NODEINFO_APP);
    resetTransmitHistory();
    TEST_ASSERT_EQUAL_UINT32(0, transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_NODEINFO_APP));
}

static void test_clear_forgets_every_stamp()
{
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_NODEINFO_APP);
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_POSITION_APP);
    transmitHistory->clear();
    TEST_ASSERT_EQUAL_UINT32(0, transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_NODEINFO_APP));
    TEST_ASSERT_EQUAL_UINT32(0, transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_POSITION_APP));
}

// --- Boot holdoff ---
// These drive the injected uptime clock from zero, so a "cold boot" is the clock reset to a known uptime.

static void bootAt(uint32_t uptimeMs, bool wokeFromDeepSleep)
{
    Time::resetMonotonicForTests();
    Time::setTestMillis(uptimeMs);
    Time::serviceMonotonic();
    transmitHistory->setWokeFromDeepSleepForTest(wokeFromDeepSleep);
}

static void test_holdoff_blocks_right_after_a_cold_boot()
{
    bootAt(1000, false);
    TEST_ASSERT_TRUE(transmitHistory->inBootHoldoff());
    TEST_ASSERT_EQUAL_UINT32(TransmitHistory::BOOT_HOLDOFF_MS - 1000, transmitHistory->bootHoldoffRemainingMs());
}

static void test_holdoff_is_five_minutes()
{
    TEST_ASSERT_EQUAL_UINT32(5 * 60 * 1000, TransmitHistory::BOOT_HOLDOFF_MS);

    bootAt(TransmitHistory::BOOT_HOLDOFF_MS - 1, false);
    TEST_ASSERT_TRUE(transmitHistory->inBootHoldoff());
    TEST_ASSERT_EQUAL_UINT32(1, transmitHistory->bootHoldoffRemainingMs());

    Time::advanceTestMillis(1);
    Time::serviceMonotonic();
    TEST_ASSERT_FALSE(transmitHistory->inBootHoldoff());
    TEST_ASSERT_EQUAL_UINT32(0, transmitHistory->bootHoldoffRemainingMs());
}

static void test_holdoff_stays_over_for_the_rest_of_the_boot()
{
    bootAt(TransmitHistory::BOOT_HOLDOFF_MS, false);
    Time::advanceTestMillis(24UL * 60 * 60 * 1000);
    Time::serviceMonotonic();
    TEST_ASSERT_FALSE(transmitHistory->inBootHoldoff());
}

static void test_holdoff_skips_a_wake_from_deep_sleep()
{
    // A sensor or tracker waking on its timer is its configured duty cycle, not a reboot loop.
    bootAt(1000, true);
    TEST_ASSERT_FALSE(transmitHistory->inBootHoldoff());
    TEST_ASSERT_EQUAL_UINT32(0, transmitHistory->bootHoldoffRemainingMs());
}

static void test_holdoff_leaves_stamps_alone()
{
    // The holdoff is a separate gate: a user-triggered send still stamps, and the stamp still throttles.
    bootAt(1000, false);
    transmitHistory->setLastSentToMesh(meshtastic_PortNum_POSITION_APP);
    TEST_ASSERT_EQUAL_UINT32(1000, transmitHistory->getLastSentToMeshMillis(meshtastic_PortNum_POSITION_APP));
}

void setup()
{
    initializeTestEnvironment();

    // Wait for portduino's millis() clock to start ticking before tests run
    testDelay(10);
    testDelay(2000);

    UNITY_BEGIN();

    RUN_TEST(test_setLastSentToMesh_stores_millis);
    RUN_TEST(test_set_overwrites_previous_value);

    RUN_TEST(test_throttle_blocks_within_interval);
    RUN_TEST(test_throttle_allows_after_interval);
    RUN_TEST(test_throttle_blocks_after_set_then_zero_does_not);

    RUN_TEST(test_multiple_keys_stored_independently);

    // Singleton
    RUN_TEST(test_getInstance_returns_same_instance);
    RUN_TEST(test_getInstance_creates_global);

    // RAM only
    RUN_TEST(test_stamps_do_not_survive_a_new_instance);
    RUN_TEST(test_clear_forgets_every_stamp);

    // Boot holdoff (injected clock; keep these last)
    RUN_TEST(test_holdoff_blocks_right_after_a_cold_boot);
    RUN_TEST(test_holdoff_is_five_minutes);
    RUN_TEST(test_holdoff_stays_over_for_the_rest_of_the_boot);
    RUN_TEST(test_holdoff_skips_a_wake_from_deep_sleep);
    RUN_TEST(test_holdoff_leaves_stamps_alone);

    exit(UNITY_END());
}

void loop() {}
