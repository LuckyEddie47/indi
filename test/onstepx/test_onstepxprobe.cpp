/*
    OnStep X INDI Driver — Capability probing unit tests (Stage 2)

    Uses a socketpair-based MockDevice that maps command strings to pre-configured
    response bytes. The driver end (fds[0]) is given to OnStepXCore; the mock end
    (fds[1]) is driven by a background thread.

    OnStepXCore uses INDI::Logger internally but guards every call with
    "if (m_dev)" — setDevice() is never called here, so all logging is silently
    skipped. The tests are entirely standalone, no INDI device instance needed.

    Key: mock map keys are the command string WITHOUT the trailing '#'.
    Response values are the raw bytes to write back (include '#' for '#'-terminated
    responses; omit it for single-char responses like set-command ACKs).
*/

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <unistd.h>

#include "OnStepXCore.h"

// ---------------------------------------------------------------------------
// MockDevice — responds to LX200-framed commands from a background thread
// ---------------------------------------------------------------------------
class MockDevice
{
public:
    // Add a pre-configured response for a command.
    // key  = command bytes WITHOUT trailing '#' (e.g. ":GVP")
    // resp = raw bytes to write back; include '#' for '#'-terminated replies,
    //        omit it for single-char ACKs or binary payloads.
    void setResponse(const std::string &key, const std::string &resp)
    {
        m_responses[key] = resp;
    }

    // Open socketpair, start background serve thread.
    // Returns the fd that should be given to OnStepXCore::setFd().
    int start()
    {
        EXPECT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, m_fds), 0);
        m_thread = std::thread([this]() { serve(); });
        return m_fds[0];
    }

    // Close driver end (signals EOF to serve thread) then join.
    void stop()
    {
        close(m_fds[0]);
        m_thread.join();
        close(m_fds[1]);
    }

private:
    // Read bytes until '#'; return everything before '#'.
    // Returns empty string on EOF.
    std::string readCmd()
    {
        std::string cmd;
        char c;
        int n;
        while ((n = read(m_fds[1], &c, 1)) == 1 && c != '#')
            cmd += c;
        if (n <= 0)
            cmd.clear();   // signal EOF to caller
        return cmd;
    }

    void serve()
    {
        while (true)
        {
            std::string cmd = readCmd();
            if (cmd.empty())
                break;   // EOF — driver end closed

            auto it = m_responses.find(cmd);
            if (it != m_responses.end())
            {
                const std::string &resp = it->second;
                if (write(m_fds[1], resp.c_str(), resp.size()) < 0)
                    break;
            }
            // Unknown command → no response; driver will time out.
            // (Short timeouts are used in tests that deliberately omit responses.)
        }
    }

    int                                m_fds[2] { -1, -1 };
    std::thread                        m_thread;
    std::map<std::string, std::string> m_responses;
};

// ---------------------------------------------------------------------------
// Shared helper: full Phase-1 response set (successful controller probe).
// Call this before start() to pre-load all Phase-1 command responses.
// ---------------------------------------------------------------------------
static void addPhase1Responses(MockDevice &dev,
                                const char *rotReply  = "R#",   // 'R'=rotator
                                const char *fa1Reply  = "1",    // focuser 1 present
                                const char *fa2Reply  = "0",    // focuser 2 absent → numFocusers=1
                                const char *featReply = "10000000#", // featureMask=1
                                const char *wxReply   = "15.0#")    // hasWeatherRead
{
    dev.setResponse(":GVP", "On-Step#");
    dev.setResponse(":GVN", "10.24c#");
    dev.setResponse(":GVD", "Mar 2026#");
    dev.setResponse(":GVT", "12:00#");
    dev.setResponse(":GVC", "Config#");
    dev.setResponse(":GX98", rotReply);
    dev.setResponse(":FA1",  fa1Reply);
    dev.setResponse(":FA2",  fa2Reply);
    dev.setResponse(":FA3",  "0");
    dev.setResponse(":GXY0", featReply);
    dev.setResponse(":GX9A", wxReply);
    dev.setResponse(":SX9A,15.0", "1");   // hasWeatherWrite
    dev.setResponse(":Gv",   "2000#");    // hasElevation
    dev.setResponse(":GX9F", "25.5#");    // hasMcuTemp
}

// Full Phase-2 response set (GEM with goto).
static void addPhase2Responses(MockDevice &dev, const char *gwReply = "GT1#")
{
    dev.setResponse(":GW",   gwReply);

    // :Gu# → 9 binary bytes all >= 0x80 (hasBinaryStatus)
    std::string bin9(9, static_cast<char>(0x80));
    dev.setResponse(":Gu",   bin9);

    dev.setResponse("$QZ?",  "!#");       // hasPec ('!' = PEC idle/ignore)
    dev.setResponse(":h?",   "0,0#");     // hasHomeSense (fields != -1)
    dev.setResponse(":GU",   "nNSK#");    // hasPPS ('S' present)
    dev.setResponse(":SU0.0","1");        // hasDUT1
    dev.setResponse(":Gm",   "E#");       // hasPierSide
}

// ---------------------------------------------------------------------------
// test_probe_full_mount
// Phase 1 + Phase 2 with all capabilities present; mountType must be GEM.
// ---------------------------------------------------------------------------
TEST(OnStepXProbeTest, test_probe_full_mount)
{
    MockDevice dev;
    addPhase1Responses(dev);
    addPhase2Responses(dev);
    int fd = dev.start();

    OnStepXCore core;
    core.setFd(fd);

    EXPECT_TRUE(core.probeController());
    EXPECT_TRUE(core.probeMount());

    const Capabilities &cap = core.caps();
    EXPECT_TRUE(cap.isOnStepX);
    EXPECT_TRUE(cap.hasRotator);
    EXPECT_FALSE(cap.hasDerotator);
    EXPECT_EQ(cap.numFocusers, 1);
    EXPECT_EQ(cap.featureMask, 1u);
    EXPECT_TRUE(cap.hasWeatherRead);
    EXPECT_TRUE(cap.hasWeatherWrite);
    EXPECT_TRUE(cap.hasElevation);
    EXPECT_TRUE(cap.hasMcuTemp);

    EXPECT_TRUE(cap.hasMount);
    EXPECT_TRUE(cap.hasGoto);
    EXPECT_EQ(cap.mountType, MountType::GEM);
    EXPECT_TRUE(cap.hasBinaryStatus);
    EXPECT_TRUE(cap.hasPec);
    EXPECT_TRUE(cap.hasHomeSense);
    EXPECT_TRUE(cap.hasPPS);
    EXPECT_TRUE(cap.hasDUT1);
    EXPECT_TRUE(cap.hasPierSide);

    dev.stop();
}

// ---------------------------------------------------------------------------
// test_probe_not_onstepx
// :GVP# returns an unrecognised product string → probeController must return
// false at the first gate (before :GVN# is even queried).
// ---------------------------------------------------------------------------
TEST(OnStepXProbeTest, test_probe_not_onstepx)
{
    MockDevice dev;
    dev.setResponse(":GVP", "LX200#");
    int fd = dev.start();

    OnStepXCore core;
    core.setFd(fd);

    EXPECT_FALSE(core.probeController());
    EXPECT_FALSE(core.caps().isOnStepX);

    dev.stop();
}

// ---------------------------------------------------------------------------
// test_probe_classic_onstep
// :GVP# returns "On-Step" (correct) but :GVN# reports firmware "4.12f"
// (major version 4, classic OnStep) → probeController must return false.
// ---------------------------------------------------------------------------
TEST(OnStepXProbeTest, test_probe_classic_onstep)
{
    MockDevice dev;
    dev.setResponse(":GVP", "On-Step#");
    dev.setResponse(":GVN", "4.12f#");
    int fd = dev.start();

    OnStepXCore core;
    core.setFd(fd);

    EXPECT_FALSE(core.probeController());
    EXPECT_FALSE(core.caps().isOnStepX);

    dev.stop();
}

// ---------------------------------------------------------------------------
// test_probe_no_mount
// Phase 1 succeeds; :GW# returns empty '#' reply → hasMount false,
// probeMount returns false.
// ---------------------------------------------------------------------------
TEST(OnStepXProbeTest, test_probe_no_mount)
{
    MockDevice dev;
    addPhase1Responses(dev);
    dev.setResponse(":GW", "#");   // empty '#'-terminated reply

    int fd = dev.start();

    OnStepXCore core;
    core.setFd(fd);

    EXPECT_TRUE(core.probeController());
    EXPECT_FALSE(core.probeMount());
    EXPECT_FALSE(core.caps().hasMount);

    dev.stop();
}

// ---------------------------------------------------------------------------
// test_probe_no_rotator
// :GX98# returns 'N' → hasRotator and hasDerotator both false.
// ---------------------------------------------------------------------------
TEST(OnStepXProbeTest, test_probe_no_rotator)
{
    MockDevice dev;
    addPhase1Responses(dev, "N#" /* rotReply */);
    int fd = dev.start();

    OnStepXCore core;
    core.setFd(fd);

    EXPECT_TRUE(core.probeController());

    EXPECT_FALSE(core.caps().hasRotator);
    EXPECT_FALSE(core.caps().hasDerotator);

    dev.stop();
}

// ---------------------------------------------------------------------------
// test_probe_no_focusers
// :FA1# returns '0' → break immediately → numFocusers == 0.
// ---------------------------------------------------------------------------
TEST(OnStepXProbeTest, test_probe_no_focusers)
{
    MockDevice dev;
    addPhase1Responses(dev, "R#", "0" /* fa1Reply — no focuser 1 */);
    int fd = dev.start();

    OnStepXCore core;
    core.setFd(fd);

    EXPECT_TRUE(core.probeController());
    EXPECT_EQ(core.caps().numFocusers, 0);

    dev.stop();
}

// ---------------------------------------------------------------------------
// test_probe_altaz
// :GW# char[0] == 'A' → mountType == ALTAZM.
// ---------------------------------------------------------------------------
TEST(OnStepXProbeTest, test_probe_altaz)
{
    MockDevice dev;
    addPhase1Responses(dev);
    addPhase2Responses(dev, "AT1#" /* gwReply: A=AltAz, T, 1=goto */);
    int fd = dev.start();

    OnStepXCore core;
    core.setFd(fd);

    EXPECT_TRUE(core.probeController());
    EXPECT_TRUE(core.probeMount());
    EXPECT_EQ(core.caps().mountType, MountType::ALTAZM);
    EXPECT_TRUE(core.caps().hasGoto);

    dev.stop();
}

// ---------------------------------------------------------------------------
// test_probe_aux_binary
// Phase 1 only (simulates aux binary — probeMount never called).
// :FA1# and :FA2# return '1', :FA3# returns '0' → numFocusers == 2.
// :GXY0# = "11000000" → featureMask == 0b00000011 == 3.
// ---------------------------------------------------------------------------
TEST(OnStepXProbeTest, test_probe_aux_binary)
{
    MockDevice dev;
    addPhase1Responses(dev,
                       "R#",        // rotReply
                       "1",         // fa1Reply
                       "1",         // fa2Reply — also present
                       "11000000#"  // featReply: bits 0+1 set → featureMask=3
    );
    dev.setResponse(":FA3", "0");   // stop here → numFocusers=2
    int fd = dev.start();

    OnStepXCore core;
    core.setFd(fd);

    EXPECT_TRUE(core.probeController());
    EXPECT_EQ(core.caps().numFocusers, 2);
    EXPECT_EQ(core.caps().featureMask, static_cast<uint8_t>(0b00000011));

    // Phase 2 never called — mount caps remain at defaults
    EXPECT_FALSE(core.caps().hasMount);

    dev.stop();
}

// ---------------------------------------------------------------------------
// main() — defined here to override indidrivermain.c's main() from
// libindidriver.so (same pattern as test_onstepxcomm.cpp).
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
