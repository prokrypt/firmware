#include "MeshTypes.h"
#include "TestUtil.h"
#include <unity.h>

#if ARCH_PORTDUINO // portduino_config.maxtophone is what sizes the queue under test

#include "configuration.h"
#include "mesh/MeshService.h"
#include "mesh/NodeDB.h"
#include "platform/portduino/PortduinoGlue.h"
#include <cstdio>
#include <cstdlib>
#include <vector>

// Queue depth for the suite. MAX_RX_TOPHONE resolves to portduino_config.maxtophone, read when
// MeshService constructs its queue.
static const int TEST_QUEUE_LEN = 4;

// Reaches the batching state so a test can age a held batch without waiting a minute.
class BatchTestService : public MeshService
{
  public:
    void ageHeldBatch() { batchStartMs = millis() - PHONE_BATCH_MAX_WAIT_MS - 1; }
};

// Counts fromNum notifications, the BLE "new packets" signal that wakes the phone.
class NotifyCounter : public Observer<uint32_t>
{
  public:
    int calls = 0;

  protected:
    int onNotify(uint32_t) override
    {
        calls++;
        return 0;
    }
};

static BatchTestService *testService = nullptr;
static NotifyCounter *notifies = nullptr;
static MeshService *savedService = nullptr;
static int savedMaxToPhone = 0;
static meshtastic_Config_DeviceConfig_RebroadcastMode savedRebroadcastMode;
static NodeNum savedNodeNum = 0;

static meshtastic_MeshPacket basePacket(uint32_t id)
{
    meshtastic_MeshPacket p = meshtastic_MeshPacket_init_zero;
    p.from = 0x11223344;
    p.to = NODENUM_BROADCAST;
    p.id = id;
    return p;
}

static void sendPacket(const meshtastic_MeshPacket &src)
{
    meshtastic_MeshPacket *p = packetPool.allocCopy(src);
    TEST_ASSERT_NOT_NULL(p);
    service->sendToPhone(p);
}

static void send(uint32_t id, meshtastic_PortNum portnum, uint32_t requestId = 0)
{
    meshtastic_MeshPacket src = basePacket(id);
    src.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    src.decoded.portnum = portnum;
    src.decoded.request_id = requestId;
    sendPacket(src);
}

static void fillWith(meshtastic_PortNum portnum, uint32_t firstId)
{
    for (int i = 0; i < TEST_QUEUE_LEN; i++)
        send(firstId + i, portnum);
}

/// Drain the queue, returning the delivered packet ids in order.
static std::vector<uint32_t> drainIds()
{
    std::vector<uint32_t> ids;
    while (meshtastic_MeshPacket *p = service->getForPhone()) {
        ids.push_back(p->id);
        service->releaseToPool(p);
    }
    return ids;
}

static void assertIds(const std::vector<uint32_t> &expected, const char *what)
{
    const std::vector<uint32_t> actual = drainIds();
    TEST_ASSERT_EQUAL_INT_MESSAGE((int)expected.size(), (int)actual.size(), what);
    for (size_t i = 0; i < expected.size(); i++)
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected[i], actual[i], what);
}

// An ACK/NAK is the phone's only delivery confirmation, so it must displace the oldest packet
// rather than be dropped when sustained downlink keeps the queue full.
static void test_routing_response_admitted_when_queue_full(void)
{
    fillWith(meshtastic_PortNum_TELEMETRY_APP, 1);
    send(100, meshtastic_PortNum_ROUTING_APP, /*requestId=*/7);

    assertIds({2, 3, 4, 100}, "oldest telemetry should have been evicted for the routing response");
}

static void test_text_evicts_oldest_when_full(void)
{
    fillWith(meshtastic_PortNum_TELEMETRY_APP, 1);
    send(200, meshtastic_PortNum_TEXT_MESSAGE_APP);

    assertIds({2, 3, 4, 200}, "text should still evict the oldest packet");
}

static void test_low_priority_packet_still_dropped_when_full(void)
{
    fillWith(meshtastic_PortNum_TEXT_MESSAGE_APP, 1);
    send(200, meshtastic_PortNum_TELEMETRY_APP);

    assertIds({1, 2, 3, 4}, "a low-priority arrival should still be dropped on a full queue");
}

// decoded.portnum aliases encrypted.size in the payload union, so a still-encrypted packet whose
// ciphertext length happens to equal a privileged portnum must not be read as one.
static void test_encrypted_packet_is_not_classified_by_portnum(void)
{
    fillWith(meshtastic_PortNum_TELEMETRY_APP, 1);

    meshtastic_MeshPacket src = basePacket(300);
    src.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
    src.encrypted.size = meshtastic_PortNum_ROUTING_APP;
    sendPacket(src);

    assertIds({1, 2, 3, 4}, "an encrypted packet must not be classified from the aliased portnum");
}

// --- Batching for an idle phone ---

static const NodeNum OUR_NODE = 0x0A0B0C0D;

static void sendTo(uint32_t id, meshtastic_PortNum portnum, NodeNum to)
{
    meshtastic_MeshPacket src = basePacket(id);
    src.to = to;
    src.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    src.decoded.portnum = portnum;
    sendPacket(src);
}

/// Runs one notify pass and returns how many notifications it sent.
static int notifyPass()
{
    const int before = notifies->calls;
    service->notifyClientsIfDue();
    return notifies->calls - before;
}

// The baseline: with no idle marker every packet is announced at once, as before batching existed.
static void test_active_phone_is_told_about_routine_packets_at_once(void)
{
    send(1, meshtastic_PortNum_TELEMETRY_APP);
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
}

static void test_idle_phone_is_not_woken_for_routine_packets(void)
{
    service->setPhoneIdle(true);
    send(1, meshtastic_PortNum_TELEMETRY_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
}

// Many routine packets over a minute cost one wake for the whole batch, not one each.
static void test_aged_batch_is_sent_in_one_notify(void)
{
    service->setPhoneIdle(true);
    send(1, meshtastic_PortNum_POSITION_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    testService->ageHeldBatch();
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
    assertIds({1}, "the held packet is delivered when the batch goes out");

    // The next routine packet starts a new batch rather than riding the old deadline.
    send(2, meshtastic_PortNum_NODEINFO_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
}

static void test_message_goes_out_at_once_with_the_held_batch(void)
{
    service->setPhoneIdle(true);
    send(1, meshtastic_PortNum_TELEMETRY_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    send(2, meshtastic_PortNum_TEXT_MESSAGE_APP);
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
    assertIds({1, 2}, "the held reading rides along with the message");
}

static void test_ack_goes_out_at_once(void)
{
    service->setPhoneIdle(true);
    send(1, meshtastic_PortNum_ROUTING_APP, /*requestId=*/7);
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
}

static void test_reading_addressed_to_us_goes_out_at_once(void)
{
    myNodeInfo.my_node_num = OUR_NODE;
    service->setPhoneIdle(true);
    sendTo(1, meshtastic_PortNum_POSITION_APP, OUR_NODE);
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
}

// Anything queued outside sendToPhone (queue status, client notifications, an identity move) is
// not a routine reading, so it is never held.
static void test_other_phone_data_goes_out_at_once(void)
{
    service->setPhoneIdle(true);
    send(1, meshtastic_PortNum_TELEMETRY_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    service->nudgeFromNum();
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
}

// The batch must never overflow the queue and drop readings while the phone sleeps.
static void test_half_full_queue_goes_out_at_once(void)
{
    service->setPhoneIdle(true);
    for (int i = 0; i < TEST_QUEUE_LEN / 2 - 1; i++)
        send(1 + i, meshtastic_PortNum_TELEMETRY_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    send(100, meshtastic_PortNum_TELEMETRY_APP);
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
}

static void test_phone_coming_back_gets_the_held_batch_at_once(void)
{
    service->setPhoneIdle(true);
    send(1, meshtastic_PortNum_TELEMETRY_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    service->setPhoneIdle(false);
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
}

// The idle phone drains the queue on its own after each heartbeat. A batch it already read must not
// wake it again later for an empty queue.
static void test_batch_read_on_the_phones_own_poll_sends_no_notify(void)
{
    service->setPhoneIdle(true);
    send(1, meshtastic_PortNum_TELEMETRY_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    assertIds({1}, "the phone polls and reads the held reading");
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    testService->ageHeldBatch();
    TEST_ASSERT_EQUAL_INT(0, notifyPass());

    // Nothing is left owed: the next reading starts a fresh batch, and a message still goes out at once.
    send(2, meshtastic_PortNum_POSITION_APP);
    TEST_ASSERT_EQUAL_INT(0, notifyPass());
    send(3, meshtastic_PortNum_TEXT_MESSAGE_APP);
    TEST_ASSERT_EQUAL_INT(1, notifyPass());
}

static void test_routine_classification(void)
{
    meshtastic_MeshPacket p = basePacket(1);
    p.which_payload_variant = meshtastic_MeshPacket_decoded_tag;

    const meshtastic_PortNum routine[] = {meshtastic_PortNum_POSITION_APP,   meshtastic_PortNum_NODEINFO_APP,
                                          meshtastic_PortNum_TELEMETRY_APP,  meshtastic_PortNum_NEIGHBORINFO_APP,
                                          meshtastic_PortNum_PAXCOUNTER_APP, meshtastic_PortNum_MAP_REPORT_APP};
    for (meshtastic_PortNum port : routine) {
        p.decoded.portnum = port;
        TEST_ASSERT_TRUE_MESSAGE(MeshService::isRoutineForPhone(p, OUR_NODE), "broadcast reading is routine");
        p.to = OUR_NODE;
        TEST_ASSERT_FALSE_MESSAGE(MeshService::isRoutineForPhone(p, OUR_NODE), "a reading sent to us is not");
        p.to = NODENUM_BROADCAST;
    }

    const meshtastic_PortNum urgent[] = {meshtastic_PortNum_TEXT_MESSAGE_APP,
                                         meshtastic_PortNum_ROUTING_APP,
                                         meshtastic_PortNum_ADMIN_APP,
                                         meshtastic_PortNum_ALERT_APP,
                                         meshtastic_PortNum_WAYPOINT_APP,
                                         meshtastic_PortNum_TRACEROUTE_APP,
                                         meshtastic_PortNum_TEXT_MESSAGE_COMPRESSED_APP};
    for (meshtastic_PortNum port : urgent) {
        p.decoded.portnum = port;
        TEST_ASSERT_FALSE_MESSAGE(MeshService::isRoutineForPhone(p, OUR_NODE), "never held");
    }
}

void setUp(void)
{
    savedMaxToPhone = portduino_config.maxtophone;
    savedRebroadcastMode = config.device.rebroadcast_mode;
    portduino_config.maxtophone = TEST_QUEUE_LEN;
    config.device.rebroadcast_mode = meshtastic_Config_DeviceConfig_RebroadcastMode_ALL;

    testService = new BatchTestService();
    savedService = service;
    service = testService;
    notifies = new NotifyCounter();
    notifies->observe(&service->fromNumChanged);
    savedNodeNum = myNodeInfo.my_node_num;
}

void tearDown(void)
{
    drainIds(); // the queue owns its pointers; a failed assertion longjmps past any in-test drain
    delete notifies;
    notifies = nullptr;
    myNodeInfo.my_node_num = savedNodeNum;
    service = savedService;
    delete testService;
    testService = nullptr;
    portduino_config.maxtophone = savedMaxToPhone;
    config.device.rebroadcast_mode = savedRebroadcastMode;
}

void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();

    printf("\n=== toPhoneQueue overflow policy ===\n");

    RUN_TEST(test_routing_response_admitted_when_queue_full);
    RUN_TEST(test_text_evicts_oldest_when_full);
    RUN_TEST(test_low_priority_packet_still_dropped_when_full);
    RUN_TEST(test_encrypted_packet_is_not_classified_by_portnum);

    printf("\n=== Batching routine packets for an idle phone ===\n");

    RUN_TEST(test_active_phone_is_told_about_routine_packets_at_once);
    RUN_TEST(test_idle_phone_is_not_woken_for_routine_packets);
    RUN_TEST(test_aged_batch_is_sent_in_one_notify);
    RUN_TEST(test_message_goes_out_at_once_with_the_held_batch);
    RUN_TEST(test_ack_goes_out_at_once);
    RUN_TEST(test_reading_addressed_to_us_goes_out_at_once);
    RUN_TEST(test_other_phone_data_goes_out_at_once);
    RUN_TEST(test_half_full_queue_goes_out_at_once);
    RUN_TEST(test_phone_coming_back_gets_the_held_batch_at_once);
    RUN_TEST(test_batch_read_on_the_phones_own_poll_sends_no_notify);
    RUN_TEST(test_routine_classification);

    exit(UNITY_END());
}

void loop() {}

#else // !ARCH_PORTDUINO

void setUp(void) {}
void tearDown(void) {}

void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    exit(UNITY_END());
}

void loop() {}

#endif
