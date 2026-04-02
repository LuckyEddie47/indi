/*
    OnStep X INDI Driver — Stage 8 unit tests: Tracking Control

    Tests verify the command strings sent for each tracking operation.
    OnStepXTracking is a plain-C++ helper so it can be exercised directly
    via a socketpair mock without a full INDI device context.

    Properties are initialised with a null device pointer which means
    property labels are empty — that's fine for command-level tests.
*/

#include <gtest/gtest.h>

#include "OnStepXComm.h"
#include "OnStepXTracking.h"

#include <sys/socket.h>
#include <thread>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static std::pair<int,int> makePair()
{
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0)
        throw std::runtime_error("socketpair failed");
    return {sv[0], sv[1]};
}

// Read one '#'-terminated command from fd, send reply+"#", return command.
static std::string respondOnce(int fd, const char *reply)
{
    char buf[256] {};
    int  pos = 0;
    while (pos < (int)sizeof(buf) - 1)
    {
        char c;
        if (read(fd, &c, 1) != 1) break;
        if (c == '#') break;
        buf[pos++] = c;
    }
    std::string r(reply);
    r += '#';
    write(fd, r.c_str(), r.size());
    return std::string(buf);
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------
class TrackingTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        auto [d, r] = makePair();
        m_driverFd    = d;
        m_responderFd = r;
        m_comm.setFd(m_driverFd);
        m_tracking.setComm(&m_comm);
        m_tracking.initProperties();
    }

    void TearDown() override
    {
        close(m_driverFd);
        if (m_responderFd >= 0) close(m_responderFd);
    }

    // Run fn() on a background thread that provides one reply, then join.
    std::string respondInBackground(const char *reply, std::function<void()> fn)
    {
        std::string received;
        std::thread t([&]{ received = respondOnce(m_responderFd, reply); });
        fn();
        t.join();
        return received;
    }

    OnStepXComm     m_comm;
    OnStepXTracking m_tracking;
    int             m_driverFd    { -1 };
    int             m_responderFd { -1 };
};

// ---------------------------------------------------------------------------
// Track compensation commands
// ---------------------------------------------------------------------------
TEST_F(TrackingTest, TrackCompFull_SendsTo)
{
    ISState states[3] = { ISS_ON, ISS_OFF, ISS_OFF };
    const char *names[3] = { "TRACK_COMP_FULL", "TRACK_COMP_REFRACTION", "TRACK_COMP_OFF" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_TRACK_COMP", states, const_cast<char**>(names), 3);
    });
    EXPECT_EQ(cmd, ":To");
}

TEST_F(TrackingTest, TrackCompRefraction_SendsTr)
{
    ISState states[3] = { ISS_OFF, ISS_ON, ISS_OFF };
    const char *names[3] = { "TRACK_COMP_FULL", "TRACK_COMP_REFRACTION", "TRACK_COMP_OFF" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_TRACK_COMP", states, const_cast<char**>(names), 3);
    });
    EXPECT_EQ(cmd, ":Tr");
}

TEST_F(TrackingTest, TrackCompOff_SendsTn)
{
    ISState states[3] = { ISS_OFF, ISS_OFF, ISS_ON };
    const char *names[3] = { "TRACK_COMP_FULL", "TRACK_COMP_REFRACTION", "TRACK_COMP_OFF" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_TRACK_COMP", states, const_cast<char**>(names), 3);
    });
    EXPECT_EQ(cmd, ":Tn");
}

// ---------------------------------------------------------------------------
// Track axis commands
// ---------------------------------------------------------------------------
TEST_F(TrackingTest, TrackAxisSingle_SendsT1)
{
    ISState states[2] = { ISS_ON, ISS_OFF };
    const char *names[2] = { "TRACK_AXIS_SINGLE", "TRACK_AXIS_DUAL" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_TRACK_COMP_AXES", states, const_cast<char**>(names), 2);
    });
    EXPECT_EQ(cmd, ":T1");
}

TEST_F(TrackingTest, TrackAxisDual_SendsT2)
{
    ISState states[2] = { ISS_OFF, ISS_ON };
    const char *names[2] = { "TRACK_AXIS_SINGLE", "TRACK_AXIS_DUAL" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_TRACK_COMP_AXES", states, const_cast<char**>(names), 2);
    });
    EXPECT_EQ(cmd, ":T2");
}

// ---------------------------------------------------------------------------
// AutoFlip commands
// ---------------------------------------------------------------------------
TEST_F(TrackingTest, AutoFlipOn_SendsSX95_1)
{
    ISState states[2] = { ISS_OFF, ISS_ON };
    const char *names[2] = { "AUTO_FLIP_OFF", "AUTO_FLIP_ON" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_AUTO_FLIP", states, const_cast<char**>(names), 2);
    });
    EXPECT_EQ(cmd, ":SX95,1");
}

TEST_F(TrackingTest, AutoFlipOff_SendsSX95_0)
{
    ISState states[2] = { ISS_ON, ISS_OFF };
    const char *names[2] = { "AUTO_FLIP_OFF", "AUTO_FLIP_ON" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_AUTO_FLIP", states, const_cast<char**>(names), 2);
    });
    EXPECT_EQ(cmd, ":SX95,0");
}

// ---------------------------------------------------------------------------
// Preferred pier side commands
// ---------------------------------------------------------------------------
TEST_F(TrackingTest, PreferredPierWest_SendsSX96_W)
{
    ISState states[3] = { ISS_ON, ISS_OFF, ISS_OFF };
    const char *names[3] = { "PIER_WEST", "PIER_EAST", "PIER_BEST" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_PREFERRED_PIER", states, const_cast<char**>(names), 3);
    });
    EXPECT_EQ(cmd, ":SX96,W");
}

TEST_F(TrackingTest, PreferredPierEast_SendsSX96_E)
{
    ISState states[3] = { ISS_OFF, ISS_ON, ISS_OFF };
    const char *names[3] = { "PIER_WEST", "PIER_EAST", "PIER_BEST" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_PREFERRED_PIER", states, const_cast<char**>(names), 3);
    });
    EXPECT_EQ(cmd, ":SX96,E");
}

TEST_F(TrackingTest, PreferredPierBest_SendsSX96_B)
{
    ISState states[3] = { ISS_OFF, ISS_OFF, ISS_ON };
    const char *names[3] = { "PIER_WEST", "PIER_EAST", "PIER_BEST" };

    auto cmd = respondInBackground("1", [&]{
        m_tracking.handleSwitch("OSX_PREFERRED_PIER", states, const_cast<char**>(names), 3);
    });
    EXPECT_EQ(cmd, ":SX96,B");
}

// ---------------------------------------------------------------------------
// SetTrackRate — verify command format produced by snprintf
// ---------------------------------------------------------------------------
TEST(TrackRate, RACommandFormat)
{
    char cmd[48];
    snprintf(cmd, sizeof(cmd), ":RA%f#", 15.0411);
    EXPECT_EQ(std::string(cmd).substr(0, 3), ":RA");
    EXPECT_NE(std::string(cmd).find("15.041"), std::string::npos);
}

TEST(TrackRate, DECommandFormat)
{
    char cmd[48];
    snprintf(cmd, sizeof(cmd), ":RE%f#", 0.0);
    EXPECT_EQ(std::string(cmd).substr(0, 3), ":RE");
    EXPECT_NE(std::string(cmd).find("0.000"), std::string::npos);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
