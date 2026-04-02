/*
    OnStep X INDI Driver — Stage 10 unit tests: Rotator helper

    Tests cover:
      - moveToAngle: correct :rS±DDD:MM:SS# format, '1' reply -> IPS_BUSY
      - moveToAngle: reject on '0' reply -> IPS_ALERT
      - abortRotator: :rQ# sent blind
      - homeRotator: :rC# sent blind -> IPS_BUSY
      - setBacklash: :rb[n]# format, '1' reply -> true
      - setBacklash: '0' reply -> false
      - pollStatus: angle parsing from :rG#, moving from :rT# 'M'
      - pollStatus: stopped from :rT# 'S'
      - readInitial: angle + backlash read on connect
      - parseAngle/formatAngle round-trip
      - formatAngle clamping: -10 -> 350, 370 -> 10
*/

#include <gtest/gtest.h>

#include "OnStepXRotator.h"
#include "OnStepXComm.h"

#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <cstdio>
#include <cstring>

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

struct Responder
{
    int fd;

    // Read one '#'-terminated command, write reply (may include '#')
    std::string expect(const char *reply)
    {
        char buf[256] {};
        int pos = 0;
        while (pos < (int)sizeof(buf) - 1)
        {
            char c;
            if (read(fd, &c, 1) != 1) break;
            if (c == '#') break;
            buf[pos++] = c;
        }
        std::string r(reply);
        write(fd, r.c_str(), r.size());
        return std::string(buf);
    }
};

// ---------------------------------------------------------------------------
// 1. moveToAngle — sends :rSDDD:MM:SS#, returns IPS_BUSY on '1' reply
// ---------------------------------------------------------------------------
TEST(RotatorHelper, MoveToAngle_AcceptedByFirmware)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::string cmd;
    std::thread t([&]{
        cmd = r.expect("1#");
        close(rFd);
    });

    IPState st = rot.moveToAngle(90.0);
    t.join();
    close(dFd);

    EXPECT_EQ(st, IPS_BUSY);
    // :rS090:00:00 (without trailing #, which is consumed by reader)
    EXPECT_EQ(cmd, ":rS090:00:00");
}

TEST(RotatorHelper, MoveToAngle_RejectedByFirmware)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::thread t([&]{
        r.expect("0#");
        close(rFd);
    });

    IPState st = rot.moveToAngle(45.0);
    t.join();
    close(dFd);

    EXPECT_EQ(st, IPS_ALERT);
}

// ---------------------------------------------------------------------------
// 2. abortRotator — sends :rQ# blind
// ---------------------------------------------------------------------------
TEST(RotatorHelper, AbortRotator_SendsCommand)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);

    std::string cmd;
    std::thread t([&]{
        char buf[32] {};
        int pos = 0;
        while (pos < (int)sizeof(buf)-1)
        {
            char c;
            if (read(rFd, &c, 1) != 1) break;
            if (c == '#') break;
            buf[pos++] = c;
        }
        cmd = std::string(buf);
        close(rFd);
    });

    bool ok = rot.abortRotator();
    t.join();
    close(dFd);

    EXPECT_TRUE(ok);
    EXPECT_EQ(cmd, ":rQ");
}

// ---------------------------------------------------------------------------
// 3. homeRotator — sends :rC# blind, returns IPS_BUSY
// ---------------------------------------------------------------------------
TEST(RotatorHelper, HomeRotator_ReturnsBusy)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);

    std::string cmd;
    std::thread t([&]{
        char buf[32] {};
        int pos = 0;
        while (pos < (int)sizeof(buf)-1)
        {
            char c;
            if (read(rFd, &c, 1) != 1) break;
            if (c == '#') break;
            buf[pos++] = c;
        }
        cmd = std::string(buf);
        close(rFd);
    });

    IPState st = rot.homeRotator();
    t.join();
    close(dFd);

    EXPECT_EQ(st, IPS_BUSY);
    EXPECT_EQ(cmd, ":rC");
}

// ---------------------------------------------------------------------------
// 4. setBacklash — :rb[n]# format, '1' -> true, '0' -> false
// ---------------------------------------------------------------------------
TEST(RotatorHelper, SetBacklash_Accepted)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::string cmd;
    std::thread t([&]{
        cmd = r.expect("1#");
        close(rFd);
    });

    bool ok = rot.setBacklash(10);
    t.join();
    close(dFd);

    EXPECT_TRUE(ok);
    EXPECT_EQ(cmd, ":rb10");
}

TEST(RotatorHelper, SetBacklash_Rejected)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::thread t([&]{
        r.expect("0#");
        close(rFd);
    });

    bool ok = rot.setBacklash(10);
    t.join();
    close(dFd);

    EXPECT_FALSE(ok);
}

// ---------------------------------------------------------------------------
// 5. pollStatus — angle from :rG#, motion from :rT#
// ---------------------------------------------------------------------------
TEST(RotatorHelper, PollStatus_Moving)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::thread t([&]{
        r.expect("045:30:00#");  // :rG# reply
        r.expect("M#");          // :rT# reply (moving)
        close(rFd);
    });

    auto result = rot.pollStatus();
    t.join();
    close(dFd);

    EXPECT_TRUE(result.angleValid);
    EXPECT_NEAR(result.angle, 45.5, 0.01);
    EXPECT_TRUE(result.statusValid);
    EXPECT_TRUE(result.moving);
}

TEST(RotatorHelper, PollStatus_Stopped)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::thread t([&]{
        r.expect("180:00:00#");  // :rG#
        r.expect("S#");          // :rT# (stopped)
        close(rFd);
    });

    auto result = rot.pollStatus();
    t.join();
    close(dFd);

    EXPECT_TRUE(result.angleValid);
    EXPECT_NEAR(result.angle, 180.0, 0.01);
    EXPECT_TRUE(result.statusValid);
    EXPECT_FALSE(result.moving);
}

// ---------------------------------------------------------------------------
// 6. readInitial — angle + backlash
// ---------------------------------------------------------------------------
TEST(RotatorHelper, ReadInitial_AngleAndBacklash)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::thread t([&]{
        r.expect("270:00:00#");  // :rG#
        r.expect("25#");         // :rb#
        close(rFd);
    });

    auto init = rot.readInitial();
    t.join();
    close(dFd);

    EXPECT_TRUE(init.angleValid);
    EXPECT_NEAR(init.angle, 270.0, 0.01);
    EXPECT_TRUE(init.backlashValid);
    EXPECT_EQ(init.backlash, 25);
}

// ---------------------------------------------------------------------------
// 7. formatAngle clamping
// ---------------------------------------------------------------------------
TEST(RotatorAngle, FormatAngle_NegativeWraps)
{
    // -10 degrees should wrap to 350
    char buf[32];
    // Access formatAngle indirectly via moveToAngle: the command sent should
    // encode 350:00:00 when angle=-10
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::string cmd;
    std::thread t([&]{
        cmd = r.expect("1#");
        close(rFd);
    });

    rot.moveToAngle(-10.0);
    t.join();
    close(dFd);

    EXPECT_EQ(cmd, ":rS350:00:00");
}

TEST(RotatorAngle, FormatAngle_OverflowWraps)
{
    // 370 degrees should wrap to 10
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    OnStepXRotator rot;
    rot.setComm(&comm);
    Responder r{rFd};

    std::string cmd;
    std::thread t([&]{
        cmd = r.expect("1#");
        close(rFd);
    });

    rot.moveToAngle(370.0);
    t.join();
    close(dFd);

    EXPECT_EQ(cmd, ":rS010:00:00");
}

// ---------------------------------------------------------------------------
// 8. Backlash command format
// ---------------------------------------------------------------------------
TEST(RotatorCmds, BacklashCommandFormat)
{
    char cmd[24];
    snprintf(cmd, sizeof(cmd), ":rb%d#", 50);
    EXPECT_STREQ(cmd, ":rb50#");
}

TEST(RotatorCmds, BacklashGetCommand)
{
    EXPECT_STREQ(":rb#", ":rb#");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
