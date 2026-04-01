/*
    OnStep X INDI Driver — Limits/Home unit tests (Stage 6)

    OnStepXLimits protocol logic is tested via socketpair harness.
    No INDI device instance needed.
*/

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <thread>
#include <map>
#include <vector>

#include "OnStepXComm.h"
#include "OnStepXLimits.h"

// ---------------------------------------------------------------------------
// Minimal socketpair responder (same pattern as earlier stages)
// ---------------------------------------------------------------------------
class LimitsResponder
{
public:
    void addRule(const std::string &key, const std::string &reply)
    {
        m_rules[key] = reply;
    }

    void start(int fd)
    {
        m_fd     = fd;
        m_thread = std::thread([this]{ run(); });
    }

    void stop()
    {
        if (m_fd >= 0) { shutdown(m_fd, SHUT_RDWR); close(m_fd); m_fd = -1; }
        if (m_thread.joinable()) m_thread.join();
    }

    const std::vector<std::string> &cmds() const { return m_cmds; }

private:
    int         m_fd { -1 };
    std::thread m_thread;
    std::map<std::string,std::string> m_rules;
    std::vector<std::string> m_cmds;

    void run()
    {
        char buf[256];
        int  pos = 0;
        while (true)
        {
            ssize_t n = read(m_fd, buf + pos, 1);
            if (n <= 0) break;
            if (buf[pos] == '#')
            {
                buf[pos + 1] = '\0';
                std::string full(buf, pos + 1);
                m_cmds.push_back(full);
                std::string key = full.substr(1, full.size() - 2);
                std::string rep = "1";
                auto it = m_rules.find(key);
                if (it != m_rules.end()) rep = it->second;
                rep += "#";
                if (write(m_fd, rep.c_str(), rep.size()) < 0) break;
                pos = 0;
            }
            else { ++pos; if (pos >= 255) pos = 0; }
        }
    }
};

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------
class LimitsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        int sv[2];
        ASSERT_EQ(0, socketpair(AF_UNIX, SOCK_STREAM, 0, sv));
        m_driverFd = sv[0];
        m_mockFd   = sv[1];
        m_comm.setFd(m_driverFd);
        m_limits.setComm(&m_comm);
        // setDevice(nullptr) is fine for unit tests — logging suppressed
    }

    void TearDown() override
    {
        m_responder.stop();
        close(m_driverFd);
    }

    int              m_driverFd { -1 };
    int              m_mockFd   { -1 };
    OnStepXComm      m_comm;
    OnStepXLimits    m_limits;
    LimitsResponder  m_responder;
};

// ---------------------------------------------------------------------------
// Home: Find
// ---------------------------------------------------------------------------
TEST_F(LimitsTest, HomeFind_SendsHC)
{
    m_responder.start(m_mockFd);
    bool ok = m_limits.homeFind();
    m_responder.stop();

    EXPECT_TRUE(ok);
    ASSERT_GE(m_responder.cmds().size(), 1u);
    EXPECT_EQ(":hC#", m_responder.cmds()[0]);
}

TEST_F(LimitsTest, HomeFind_FailsOnNack)
{
    // Respond with '0' (NACK) — homeFind should return false
    m_responder.addRule("hC", "0");
    m_responder.start(m_mockFd);
    bool ok = m_limits.homeFind();
    m_responder.stop();
    EXPECT_FALSE(ok);
}

// ---------------------------------------------------------------------------
// Home: Set
// ---------------------------------------------------------------------------
TEST_F(LimitsTest, HomeSet_SendsHF)
{
    m_responder.start(m_mockFd);
    bool ok = m_limits.homeSet();
    m_responder.stop();

    EXPECT_TRUE(ok);
    ASSERT_GE(m_responder.cmds().size(), 1u);
    EXPECT_EQ(":hF#", m_responder.cmds()[0]);
}

// ---------------------------------------------------------------------------
// Auto-home on boot
// ---------------------------------------------------------------------------
TEST_F(LimitsTest, SetAutoHome_EnableSendsHA1)
{
    m_responder.start(m_mockFd);
    bool ok = m_limits.setAutoHome(true);
    m_responder.stop();

    EXPECT_TRUE(ok);
    ASSERT_GE(m_responder.cmds().size(), 1u);
    EXPECT_EQ(":hA1#", m_responder.cmds()[0]);
}

TEST_F(LimitsTest, SetAutoHome_DisableSendsHA0)
{
    m_responder.start(m_mockFd);
    bool ok = m_limits.setAutoHome(false);
    m_responder.stop();

    EXPECT_TRUE(ok);
    ASSERT_GE(m_responder.cmds().size(), 1u);
    EXPECT_EQ(":hA0#", m_responder.cmds()[0]);
}

// ---------------------------------------------------------------------------
// Home offsets
// ---------------------------------------------------------------------------
TEST_F(LimitsTest, WriteHomeOffsets_SendsBothAxes)
{
    m_responder.start(m_mockFd);
    bool ok = m_limits.writeHomeOffsets(120.0, -45.0);
    m_responder.stop();

    EXPECT_TRUE(ok);
    const auto &cmds = m_responder.cmds();
    ASSERT_GE(cmds.size(), 2u);

    bool hasAx1 = false, hasAx2 = false;
    for (const auto &c : cmds)
    {
        if (c.find(":hC1,") != std::string::npos) hasAx1 = true;
        if (c.find(":hC2,") != std::string::npos) hasAx2 = true;
    }
    EXPECT_TRUE(hasAx1) << "Missing :hC1,# command";
    EXPECT_TRUE(hasAx2) << "Missing :hC2,# command";
}

TEST_F(LimitsTest, WriteHomeOffsets_CorrectValues)
{
    m_responder.start(m_mockFd);
    m_limits.writeHomeOffsets(300.0, -150.0);
    m_responder.stop();

    const auto &cmds = m_responder.cmds();
    bool ax1ok = false, ax2ok = false;
    for (const auto &c : cmds)
    {
        if (c.find(":hC1,300#") != std::string::npos) ax1ok = true;
        if (c.find(":hC2,-150#") != std::string::npos) ax2ok = true;
    }
    EXPECT_TRUE(ax1ok) << "Expected :hC1,300# in: " << (cmds.empty() ? "(none)" : cmds[0]);
    EXPECT_TRUE(ax2ok) << "Expected :hC2,-150# in commands";
}

// ---------------------------------------------------------------------------
// readLimits
// ---------------------------------------------------------------------------
TEST_F(LimitsTest, ReadLimits_ParsesHorizonAndMeridian)
{
    m_responder.addRule("Gh",   "-10");
    m_responder.addRule("Go",   "89");
    m_responder.addRule("GXE9", "15");
    m_responder.addRule("GXEA", "30");
    m_responder.start(m_mockFd);

    // initProperties must be called so the PropertyNumber objects exist
    m_limits.initProperties();
    bool ok = m_limits.readLimits();
    m_responder.stop();

    EXPECT_TRUE(ok);
    EXPECT_NEAR(-10.0, m_limits.horizonLimitNP()[0].getValue(),  0.01);
    EXPECT_NEAR( 89.0, m_limits.horizonLimitNP()[1].getValue(),  0.01);
    EXPECT_NEAR( 15.0, m_limits.meridianLimitNP()[0].getValue(), 0.01);
    EXPECT_NEAR( 30.0, m_limits.meridianLimitNP()[1].getValue(), 0.01);
}

// ---------------------------------------------------------------------------
// handleNumber — horizon limits write
// ---------------------------------------------------------------------------
TEST_F(LimitsTest, HandleNumber_HorizonLimitsSendsShSo)
{
    m_limits.initProperties();
    m_responder.start(m_mockFd);

    double values[] = { -5.0, 85.0 };
    const char *names[] = { "HORIZON_MIN", "HORIZON_MAX" };
    m_limits.handleNumber("HORIZON_LIMITS", values, const_cast<char **>(names), 2);
    m_responder.stop();

    bool hasSh = false, hasSo = false;
    for (const auto &c : m_responder.cmds())
    {
        if (c.find(":Sh") != std::string::npos) hasSh = true;
        if (c.find(":So") != std::string::npos) hasSo = true;
    }
    EXPECT_TRUE(hasSh) << "Missing :Sh# horizon min command";
    EXPECT_TRUE(hasSo) << "Missing :So# horizon max command";
}

// ---------------------------------------------------------------------------
// handleNumber — meridian limits write
// ---------------------------------------------------------------------------
TEST_F(LimitsTest, HandleNumber_MeridianLimitsSendsSXE9SXEA)
{
    m_limits.initProperties();
    m_responder.start(m_mockFd);

    double values[] = { 20.0, 40.0 };
    const char *names[] = { "MERIDIAN_EAST", "MERIDIAN_WEST" };
    m_limits.handleNumber("MERIDIAN_LIMITS", values, const_cast<char **>(names), 2);
    m_responder.stop();

    bool hasSXE9 = false, hasSXEA = false;
    for (const auto &c : m_responder.cmds())
    {
        if (c.find(":SXE9,") != std::string::npos) hasSXE9 = true;
        if (c.find(":SXEA,") != std::string::npos) hasSXEA = true;
    }
    EXPECT_TRUE(hasSXE9) << "Missing :SXE9,# command";
    EXPECT_TRUE(hasSXEA) << "Missing :SXEA,# command";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
