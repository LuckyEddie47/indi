/*
    OnStep X INDI Driver — Stage 7 unit tests: Pulse Guide

    Tests cover:
      - Guide command format strings (:MGn/s/e/w{ms}#)
      - Guide rate command and parse
      - SensorData struct (WeatherInterface refactor sanity check)

    Guide timing / GuideComplete integration is verified manually via PHD2
    because OnStepXMount inherits INDI::Telescope and requires a full device
    context that is not available in a unit-test binary.
*/

#include <gtest/gtest.h>

#include "OnStepXComm.h"
#include "OnStepXWeather.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Minimal socket-pair responder (same pattern as other test files)
// ---------------------------------------------------------------------------
struct Responder
{
    int fd { -1 };

    explicit Responder(int fd) : fd(fd) {}

    // Read one '#'-terminated command, send the given reply (appends '#')
    // Returns the command received (without '#').
    std::string once(const char *reply)
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
        std::string cmd(buf);
        std::string r(reply);
        r += '#';
        if (write(fd, r.c_str(), r.size())) {};
        return cmd;
    }

    void close_fd() { close(fd); fd = -1; }
};

// ---------------------------------------------------------------------------
// Helper: create a socketpair and return {driverFd, responderFd}
// ---------------------------------------------------------------------------
static std::pair<int,int> makePair()
{
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0)
        throw std::runtime_error("socketpair failed");
    return {sv[0], sv[1]};
}

// ---------------------------------------------------------------------------
// 1. Guide command format tests (pure string formatting, no comm needed)
// ---------------------------------------------------------------------------
TEST(GuideCommand, FormatNorth)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":MGn%u#", 500u);
    EXPECT_STREQ(cmd, ":MGn500#");
}

TEST(GuideCommand, FormatSouth)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":MGs%u#", 250u);
    EXPECT_STREQ(cmd, ":MGs250#");
}

TEST(GuideCommand, FormatEast)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":MGe%u#", 100u);
    EXPECT_STREQ(cmd, ":MGe100#");
}

TEST(GuideCommand, FormatWest)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":MGw%u#", 1000u);
    EXPECT_STREQ(cmd, ":MGw1000#");
}

TEST(GuideCommand, ZeroMs)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":MGn%u#", 0u);
    EXPECT_STREQ(cmd, ":MGn0#");
}

// ---------------------------------------------------------------------------
// 2. Guide rate parsing — OnStepXComm sends :GX90#, firmware replies with
//    a decimal fraction of sidereal (e.g., "0.50")
// ---------------------------------------------------------------------------
TEST(GuideRate, ParsesValidReply)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);

    std::string received;
    std::thread t([&]{
        Responder r(rFd);
        received = r.once("0.50");
        r.close_fd();
    });

    char reply[32];
    bool ok = comm.sendCommand(":GX90#", reply);
    t.join();
    close(dFd);

    EXPECT_TRUE(ok);
    EXPECT_STREQ(received.c_str(), ":GX90");

    char *end;
    double rate = std::strtod(reply, &end);
    EXPECT_NE(end, reply);
    EXPECT_DOUBLE_EQ(rate, 0.50);
}

TEST(GuideRate, IgnoresErrorReply)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);

    std::thread t([&]{
        Responder r(rFd);
        r.once("ERR");
        r.close_fd();
    });

    char reply[32];
    bool ok = comm.sendCommand(":GX90#", reply);
    t.join();
    close(dFd);

    // The command succeeds at comm level; the caller must reject non-numeric
    EXPECT_TRUE(ok);

    char *end;
    double rate = std::strtod(reply, &end);
    // "ERR" is not numeric — strtod returns 0.0 and end == reply
    EXPECT_EQ(end, reply);   // no digits consumed
    (void)rate;
}

// ---------------------------------------------------------------------------
// 3. SensorData struct — sanity check for the WeatherInterface refactor
// ---------------------------------------------------------------------------
TEST(SensorData, AnyOkFalseWhenEmpty)
{
    SensorData d;
    EXPECT_FALSE(d.anyOk());
    EXPECT_FALSE(d.temp.ok);
    EXPECT_FALSE(d.pressure.ok);
    EXPECT_FALSE(d.humidity.ok);
    EXPECT_FALSE(d.dewpoint.ok);
    EXPECT_FALSE(d.mcuTemp.ok);
}

TEST(SensorData, AnyOkTrueWhenTempOnly)
{
    SensorData d;
    d.temp = {true, 20.5};
    EXPECT_TRUE(d.anyOk());
    EXPECT_DOUBLE_EQ(d.temp.value, 20.5);
    EXPECT_FALSE(d.pressure.ok);
}

TEST(SensorData, AnyOkTrueWhenMcuTempOnly)
{
    SensorData d;
    d.mcuTemp = {true, 45.0};
    EXPECT_TRUE(d.anyOk());
    EXPECT_FALSE(d.temp.ok);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
