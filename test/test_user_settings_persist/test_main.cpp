// User-initiated settings must survive a reboot. Each test drives the real local admin path
// (AdminModule::handleReceivedProtobuf, from == 0) the way a phone app does, then "reboots" a fresh
// NodeDB from the files on flash and checks what came back.
//
// The no-auto-writes fork removed every automatic save (deep sleep, node DB autosave, boot catch-all),
// so a user change now reaches flash only through the admin save itself. These tests pin that it does.
#include "MeshTypes.h" // Include BEFORE TestUtil.h
#include "TestUtil.h"
#include <unity.h>

#if defined(ARCH_PORTDUINO)
#define USP_TEST_ENTRY extern "C"
#else
#define USP_TEST_ENTRY
#endif

#include "FSCommon.h" // defines FSCom; must precede the feature guard below

#if defined(FSCom) && !(MESHTASTIC_EXCLUDE_PKI_KEYGEN || MESHTASTIC_EXCLUDE_PKI)

#include "FlashGuard.h"
#include "main.h" // rebootAtMsec
#include "mesh/Channels.h"
#include "mesh/NodeDB.h"
#include "modules/NodeInfoModule.h"
#include "support/AdminModuleTestShim.h"
#include "support/MockMeshService.h"
#include <cstring>
#include <string>
#include <unistd.h>

namespace
{

MockMeshService *mockSvc = nullptr;
AdminModuleTestShim *admin = nullptr;

// A real boot starts with a zeroed nodeDatabase; in-process the global keeps the previous boot's
// vector (the decode callback appends), so reset it first. RAM copies of every segment are scribbled
// over so a value can only survive by coming back from flash.
void rebootNodeDB()
{
    strncpy(owner.long_name, "RAM ONLY", sizeof(owner.long_name));
    strncpy(owner.short_name, "RAM", sizeof(owner.short_name));
    config.display.screen_on_secs = 1;
    moduleConfig.telemetry.environment_update_interval = 1;
    nodeDatabase.version = 0;
    nodeDatabase.nodes.clear();
    NodeDB *rebooted = new NodeDB();
    delete nodeDB;
    nodeDB = rebooted;
}

void sendAdmin(meshtastic_AdminMessage &m)
{
    meshtastic_MeshPacket mp = meshtastic_MeshPacket_init_zero;
    mp.from = 0; // local client (BLE/USB/TCP)
    mp.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    admin->handleReceivedProtobuf(mp, &m);
    admin->drainReply();
}

void sendBeginEdit()
{
    meshtastic_AdminMessage m = meshtastic_AdminMessage_init_zero;
    m.which_payload_variant = meshtastic_AdminMessage_begin_edit_settings_tag;
    m.begin_edit_settings = true;
    sendAdmin(m);
}

void sendCommitEdit()
{
    meshtastic_AdminMessage m = meshtastic_AdminMessage_init_zero;
    m.which_payload_variant = meshtastic_AdminMessage_commit_edit_settings_tag;
    m.commit_edit_settings = true;
    sendAdmin(m);
}

void sendSetOwner(const char *longName, const char *shortName)
{
    meshtastic_AdminMessage m = meshtastic_AdminMessage_init_zero;
    m.which_payload_variant = meshtastic_AdminMessage_set_owner_tag;
    strncpy(m.set_owner.long_name, longName, sizeof(m.set_owner.long_name) - 1);
    strncpy(m.set_owner.short_name, shortName, sizeof(m.set_owner.short_name) - 1);
    sendAdmin(m);
}

void sendSetDisplay(uint32_t screenOnSecs)
{
    meshtastic_AdminMessage m = meshtastic_AdminMessage_init_zero;
    m.which_payload_variant = meshtastic_AdminMessage_set_config_tag;
    m.set_config.which_payload_variant = meshtastic_Config_display_tag;
    m.set_config.payload_variant.display = config.display;
    m.set_config.payload_variant.display.screen_on_secs = screenOnSecs;
    sendAdmin(m);
}

void sendSetTelemetry(uint32_t envIntervalSecs)
{
    meshtastic_AdminMessage m = meshtastic_AdminMessage_init_zero;
    m.which_payload_variant = meshtastic_AdminMessage_set_module_config_tag;
    m.set_module_config.which_payload_variant = meshtastic_ModuleConfig_telemetry_tag;
    m.set_module_config.payload_variant.telemetry = moduleConfig.telemetry;
    m.set_module_config.payload_variant.telemetry.environment_update_interval = envIntervalSecs;
    sendAdmin(m);
}

void sendGetOwner()
{
    meshtastic_AdminMessage m = meshtastic_AdminMessage_init_zero;
    m.which_payload_variant = meshtastic_AdminMessage_get_owner_request_tag;
    m.get_owner_request = true;
    sendAdmin(m);
}

// A set region arms boot keygen, and a node DB is only persisted once we hold a key.
void establishKeyedBaseline()
{
    config.lora.region = meshtastic_Config_LoRaConfig_RegionCode_US;
    TEST_ASSERT_TRUE(nodeDB->saveToDisk(SEGMENT_CONFIG));
    rebootNodeDB();
    TEST_ASSERT_EQUAL(32, owner.public_key.size);
}

void assertOwnerAfterReboot(const char *longName, const char *shortName)
{
    rebootNodeDB();
    TEST_ASSERT_EQUAL_STRING(longName, owner.long_name);
    TEST_ASSERT_EQUAL_STRING(shortName, owner.short_name);
    // Clients show our own node entry, so it must agree with devicestate after the reboot too.
    const meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
    TEST_ASSERT_NOT_NULL(self);
    TEST_ASSERT_EQUAL_STRING(longName, self->long_name);
    TEST_ASSERT_EQUAL_STRING(shortName, self->short_name);
}

} // namespace

void setUp(void)
{
    mockSvc = new MockMeshService();
    service = mockSvc;
    admin = new AdminModuleTestShim();
    // reloadOwner() only checks presence while a transaction is open; tests without one stay inside it.
    nodeInfoModule = reinterpret_cast<NodeInfoModule *>(1);
}

void tearDown(void)
{
    nodeInfoModule = nullptr;
    delete admin;
    admin = nullptr;
    service = nullptr;
    delete mockSvc;
    mockSvc = nullptr;
    rebootAtMsec = 0; // commit schedules a reboot; nothing here acts on it
}

// What the Android and iOS apps send when the user saves the User page.
static void test_setOwner_inEditTransaction_survivesReboot(void)
{
    establishKeyedBaseline();
    sendBeginEdit();
    sendSetOwner("Tx Long Name", "TXLN");
    sendCommitEdit();
    TEST_ASSERT_FALSE(admin->editTransactionOpen());
    assertOwnerAfterReboot("Tx Long Name", "TXLN");
}

// A client that opens a transaction and never commits: the next admin message after the idle window
// commits what it applied.
static void test_setOwner_abandonedTransaction_committedByNextAdmin(void)
{
    sendBeginEdit();
    sendSetOwner("Abandoned Name", "ABND");
    admin->ageEditTransaction();
    sendGetOwner();
    TEST_ASSERT_FALSE(admin->editTransactionOpen());
    assertOwnerAfterReboot("Abandoned Name", "ABND");
}

static void test_setConfig_inEditTransaction_survivesReboot(void)
{
    sendBeginEdit();
    sendSetDisplay(123);
    sendSetTelemetry(4321);
    sendCommitEdit();
    rebootNodeDB();
    TEST_ASSERT_EQUAL_UINT32(123, config.display.screen_on_secs);
    TEST_ASSERT_EQUAL_UINT32(4321, moduleConfig.telemetry.environment_update_interval);
}

static void test_setConfig_withoutTransaction_survivesReboot(void)
{
    sendSetDisplay(77);
    sendSetTelemetry(1800);
    rebootNodeDB();
    TEST_ASSERT_EQUAL_UINT32(77, config.display.screen_on_secs);
    TEST_ASSERT_EQUAL_UINT32(1800, moduleConfig.telemetry.environment_update_interval);
}

// A user save that cannot write (here a directory squats on device.proto's temp file, so SafeFile can't open
// it) must tell the client and stay owed: the retry at the user's reboot lands it once flash takes writes again.
static void test_failedUserSave_notifiesAndRetriesAtReboot(void)
{
    const char *blocker = "/prefs/device.proto.tmp";
    const char *blockerFile = "/prefs/device.proto.tmp/x";
    FSCom.mkdir(blocker);
    File f = FSCom.open(blockerFile, FILE_O_WRITE);
    TEST_ASSERT_TRUE(f);
    f.print("x");
    f.close();

    sendBeginEdit();
    sendSetOwner("Retry Name", "RTRY");
    sendCommitEdit();
    TEST_ASSERT_GREATER_THAN_UINT32(0, mockSvc->notificationCount);

    FSCom.remove(blockerFile);
    // Portduino's FS::rmdir() calls unlink(), which can't remove a directory; go to the host FS instead.
    TEST_ASSERT_EQUAL(0, ::rmdir((std::string(portduinoVFS->mountpoint()) + blocker).c_str()));
    TEST_ASSERT_FALSE(FSCom.exists(blocker));
    {
        FlashGuard::Scope userWrite("test reboot");
        nodeDB->retryFailedUserSave();
    }
    assertOwnerAfterReboot("Retry Name", "RTRY");
}

// A second reboot with no user action in between must not lose or rewrite anything.
static void test_secondReboot_keepsOwner(void)
{
    assertOwnerAfterReboot("Retry Name", "RTRY");
}

USP_TEST_ENTRY void setup()
{
    initializeTestEnvironment();
    nodeDB = new NodeDB(); // first boot on the pristine per-suite sandbox

    UNITY_BEGIN();
    RUN_TEST(test_setOwner_inEditTransaction_survivesReboot);
    RUN_TEST(test_setOwner_abandonedTransaction_committedByNextAdmin);
    RUN_TEST(test_setConfig_inEditTransaction_survivesReboot);
    RUN_TEST(test_setConfig_withoutTransaction_survivesReboot);
    RUN_TEST(test_failedUserSave_notifiesAndRetriesAtReboot);
    RUN_TEST(test_secondReboot_keepsOwner);
    exit(UNITY_END());
}

USP_TEST_ENTRY void loop() {}

#else // !FSCom || PKI excluded

void setUp(void) {}
void tearDown(void) {}

USP_TEST_ENTRY void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    exit(UNITY_END());
}

USP_TEST_ENTRY void loop() {}

#endif
