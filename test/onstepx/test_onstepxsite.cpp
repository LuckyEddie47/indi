/*
    OnStep X INDI Driver — Site/time unit tests (Stage 5)

    OnStepXSite::writeLocation / readLocation / writeTime are tested by
    replaying the commands it sends to the controller via the socketpair
    harness established in earlier stages.

    All tests are self-contained: no real controller required.
*/

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>
#include <map>

#include "OnStepXComm.h"
#include "OnStepXSite.h"

// ---------------------------------------------------------------------------
// Minimal responder — fixed command -> reply map (same harness as Stage 1/2)
// ---------------------------------------------------------------------------
class SiteResponder
{
public:
    // Map cmd (without the leading ':' and trailing '#') -> reply string
    // (without trailing '#').
    void addRule(const std::string &cmd, const std::string &reply)
    {
        m_rules[cmd] = reply;
    }

    // Start background thread listening on fd.
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

    // Last command received (full, including ':' and '#').
    std::string lastCmd() const { return m_lastCmd; }
    // All commands received in order.
    const std::vector<std::string> &cmds() const { return m_cmds; }

private:
    int         m_fd { -1 };
    std::thread m_thread;
    std::map<std::string,std::string> m_rules;
    std::string m_lastCmd;
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
                std::string full(buf, pos + 1);   // includes '#'
                m_lastCmd = full;
                m_cmds.push_back(full);

                // Strip ':' prefix and '#' suffix to look up rule.
                std::string key = full.substr(1, full.size() - 2);
                std::string rep = "1";             // default ACK
                auto it = m_rules.find(key);
                if (it != m_rules.end())
                    rep = it->second;
                rep += "#";
                if (write(m_fd, rep.c_str(), rep.size()) < 0) break;
                pos = 0;
            }
            else
            {
                ++pos;
                if (pos >= 255) pos = 0;
            }
        }
    }
};

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------
class SiteTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        int sv[2];
        ASSERT_EQ(0, socketpair(AF_UNIX, SOCK_STREAM, 0, sv));
        m_driverFd = sv[0];
        m_mockFd   = sv[1];

        m_comm.setFd(m_driverFd);
        m_site.setComm(&m_comm);
        // m_site.setDevice() left as nullptr — that's fine for unit tests.
    }

    void TearDown() override
    {
        m_responder.stop();
        close(m_driverFd);
    }

    int           m_driverFd { -1 };
    int           m_mockFd   { -1 };
    OnStepXComm   m_comm;
    OnStepXSite   m_site;
    SiteResponder m_responder;
};

// ---------------------------------------------------------------------------
// writeLocation tests
// ---------------------------------------------------------------------------

TEST_F(SiteTest, WriteLocation_SendsLatLonElev)
{
    m_responder.start(m_mockFd);

    // All three commands should be ACKed with '1'.
    bool ok = m_site.writeLocation(51.5, 0.0, 100.0);
    m_responder.stop();

    EXPECT_TRUE(ok);

    // Must have sent at least :St#, :Sg#, :Sv#
    const auto &cmds = m_responder.cmds();
    ASSERT_GE(cmds.size(), 3u);

    bool hasSt = false, hasSg = false, hasSv = false;
    for (const auto &c : cmds)
    {
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 't') hasSt = true;
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 'g') hasSg = true;
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 'v') hasSv = true;
    }
    EXPECT_TRUE(hasSt) << "Missing :St# (latitude) command";
    EXPECT_TRUE(hasSg) << "Missing :Sg# (longitude) command";
    EXPECT_TRUE(hasSv) << "Missing :Sv# (elevation) command";
}

TEST_F(SiteTest, WriteLocation_LatCommandFormat)
{
    m_responder.start(m_mockFd);
    m_site.writeLocation(51.5081, 0.0, 0.0);
    m_responder.stop();

    // Find the :St# command and check it starts with '+' (Northern hemisphere)
    bool found = false;
    for (const auto &c : m_responder.cmds())
    {
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 't')
        {
            EXPECT_EQ('+', c[3]) << "Northern latitude should start with '+'";
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(SiteTest, WriteLocation_NegativeLatitude)
{
    m_responder.start(m_mockFd);
    m_site.writeLocation(-33.87, 151.21, 50.0);
    m_responder.stop();

    bool found = false;
    for (const auto &c : m_responder.cmds())
    {
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 't')
        {
            EXPECT_EQ('-', c[3]) << "Southern latitude should start with '-'";
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(SiteTest, WriteLocation_LongitudeConversion)
{
    // INDI lon 151.21 East -> OnStepX West-positive = 360 - 151.21 = 208.79
    // :Sg# should encode 208 degrees.
    m_responder.start(m_mockFd);
    m_site.writeLocation(-33.87, 151.21, 0.0);
    m_responder.stop();

    bool found = false;
    for (const auto &c : m_responder.cmds())
    {
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 'g')
        {
            // Command is ":Sg208:..." — degree field is characters 3-5
            std::string deg = c.substr(3, 3);
            EXPECT_EQ("208", deg) << "Expected 208 degrees West for 151.21 East; got: " << c;
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(SiteTest, WriteLocation_ReturnsFalseOnNack)
{
    // Respond to the first command (:St#) with '0' (NACK) by using a
    // separate write on the mock fd before starting the responder thread.
    // We send a NACK directly for the latitude command.
    //
    // Strategy: start the responder, but override its default '1' reply
    // by pre-seeding a NACK for any :St...# command.  The SiteResponder
    // uses prefix matching via the full key — instead, we intercept by
    // writing a '0#' reply on the socket before writeLocation can read it,
    // then let the socket drain.
    //
    // Simpler: use a second socketpair where the "controller" side sends
    // a canned '0#' for the first request and then hangs up.
    close(m_mockFd);   // close the responder fd from SetUp
    int sv[2];
    ASSERT_EQ(0, socketpair(AF_UNIX, SOCK_STREAM, 0, sv));
    m_driverFd = sv[0];
    m_mockFd   = sv[1];
    m_comm.setFd(m_driverFd);

    // Background: consume the :St# command, reply '0#', then close.
    std::thread t([&]()
    {
        char buf[64];
        int pos = 0;
        while (pos < 63)
        {
            ssize_t n = read(m_mockFd, buf + pos, 1);
            if (n <= 0) break;
            if (buf[pos] == '#') { buf[++pos] = '\0'; break; }
            ++pos;
        }
        const char nack[] = "0#";
        if (write(m_mockFd, nack, sizeof(nack) - 1) < 0) {}
        close(m_mockFd);
        m_mockFd = -1;
    });

    bool ok = m_site.writeLocation(51.508, 0.0, 0.0);
    t.join();

    EXPECT_FALSE(ok);
}

// ---------------------------------------------------------------------------
// readLocation tests
// ---------------------------------------------------------------------------

TEST_F(SiteTest, ReadLocation_ParsesLatLon)
{
    // :GtH# -> "+51:30:29.2"  (51.508 deg N)
    // :GgH# -> "000:00:00.0"  (0 deg West -> 360 E -> normalises to 0)
    // :Gv#  -> "100"
    m_responder.addRule("GtH", "+51:30:29.2");
    m_responder.addRule("GgH", "000:00:00.0");
    m_responder.addRule("Gv",  "100");
    m_responder.start(m_mockFd);

    double lat = 0, lon = 0, elev = 0;
    bool ok = m_site.readLocation(lat, lon, elev);
    m_responder.stop();

    EXPECT_TRUE(ok);
    EXPECT_NEAR(51.508, lat,  0.01);
    EXPECT_NEAR(0.0,    lon,  0.001);
    EXPECT_NEAR(100.0,  elev, 1.0);
}

TEST_F(SiteTest, ReadLocation_LongitudeEastConversion)
{
    // OnStepX stores 208:47 West -> should come back as ~151.21 East
    m_responder.addRule("GtH", "-33:52:12.0");
    m_responder.addRule("GgH", "208:47:24.0");
    m_responder.addRule("Gv",  "50");
    m_responder.start(m_mockFd);

    double lat = 0, lon = 0, elev = 0;
    bool ok = m_site.readLocation(lat, lon, elev);
    m_responder.stop();

    EXPECT_TRUE(ok);
    EXPECT_NEAR(151.21, lon, 0.01);
}

// ---------------------------------------------------------------------------
// writeTime tests
// ---------------------------------------------------------------------------

TEST_F(SiteTest, WriteTime_SendsThreeCommands)
{
    m_responder.start(m_mockFd);

    ln_date utc;
    utc.years = 2026; utc.months = 4; utc.days = 1;
    utc.hours = 12;   utc.minutes = 0; utc.seconds = 0.0;

    bool ok = m_site.writeTime(&utc, 10.0);   // UTC+10
    m_responder.stop();

    EXPECT_TRUE(ok);

    bool hasSG = false, hasSL = false, hasSC = false;
    for (const auto &c : m_responder.cmds())
    {
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 'G') hasSG = true;
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 'L') hasSL = true;
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 'C') hasSC = true;
    }
    EXPECT_TRUE(hasSG) << "Missing :SG# (UTC offset) command";
    EXPECT_TRUE(hasSL) << "Missing :SL# (local time) command";
    EXPECT_TRUE(hasSC) << "Missing :SC# (local date) command";
}

TEST_F(SiteTest, WriteTime_UTCOffsetFormat)
{
    m_responder.start(m_mockFd);

    ln_date utc { 2026, 4, 1, 0, 0, 0.0 };
    m_site.writeTime(&utc, -5.0);   // UTC-5 (EST)
    m_responder.stop();

    for (const auto &c : m_responder.cmds())
    {
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 'G')
        {
            // Should contain '-05' or '-5'
            EXPECT_NE(std::string::npos, c.find('-'))
                << "Negative UTC offset should have '-': " << c;
        }
    }
}

TEST_F(SiteTest, WriteTime_PositiveUTCOffsetFormat)
{
    m_responder.start(m_mockFd);

    ln_date utc { 2026, 4, 1, 0, 0, 0.0 };
    m_site.writeTime(&utc, +10.0);
    m_responder.stop();

    for (const auto &c : m_responder.cmds())
    {
        if (c.size() >= 3 && c[1] == 'S' && c[2] == 'G')
        {
            EXPECT_NE(std::string::npos, c.find('+'))
                << "Positive UTC offset should have '+': " << c;
        }
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
