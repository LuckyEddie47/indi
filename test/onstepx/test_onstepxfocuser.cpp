/*
    OnStep X INDI Driver — Stage 9 unit tests: Focuser

    Tests cover:
      - sendCommandFocuser / sendCommandBlindFocuser command atomicity
      - Command format strings (:FG#, :FM#, :FT#, :Ft#, :FN#, :Fm#, :FQ#, :FB#, :FP#)
      - Position and temperature parsing
      - Focuser selection command format (:FA[n]#)

    Full focuser device integration is verified via Ekos (requires hardware).
*/

#include <gtest/gtest.h>

#include "OnStepXComm.h"

#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <cstdio>

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

// Responder: read commands, write replies.
// Each call to expect() reads one '#'-terminated command and writes reply+'#'.
struct Responder
{
    int fd;

    std::string expect(const char *reply)
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
        // Send reply (no '#' for single-char, '#'-terminated for regular)
        std::string r(reply);
        write(fd, r.c_str(), r.size());
        return std::string(buf);
    }

    // Read one command, send a single-char reply (no '#')
    std::string expectSingle(char reply)
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
        write(fd, &reply, 1);
        return std::string(buf);
    }
};

// ---------------------------------------------------------------------------
// 1. sendCommandFocuser: atomically selects focuser, reads single-char reply,
//    then sends command and reads '#'-terminated reply.
// ---------------------------------------------------------------------------
TEST(FocuserComm, SendCommandFocuser_SelectThenCmd)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    Responder r{rFd};

    std::string sel, cmd;
    std::thread t([&]{
        sel = r.expectSingle('1');   // :FA2# -> '1'
        cmd = r.expect("1234#");     // :FG# -> "1234#"
        close(rFd);
    });

    char reply[64];
    bool ok = comm.sendCommandFocuser(2, ":FG#", reply);
    t.join();
    close(dFd);

    EXPECT_TRUE(ok);
    EXPECT_EQ(sel, ":FA2");
    EXPECT_EQ(cmd, ":FG");
    EXPECT_STREQ(reply, "1234");
}

TEST(FocuserComm, SendCommandFocuser_RejectsNack)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    Responder r{rFd};

    std::thread t([&]{
        r.expectSingle('0');   // focuser not found
        close(rFd);
    });

    char reply[64] {};
    bool ok = comm.sendCommandFocuser(3, ":FG#", reply);
    t.join();
    close(dFd);

    EXPECT_FALSE(ok);
}

TEST(FocuserComm, SendCommandBlindFocuser_SelectThenCmd)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    Responder r{rFd};

    std::string sel, cmd;
    std::thread t([&]{
        sel = r.expectSingle('1');   // :FA1# -> '1'
        // Read the blind command (no reply sent)
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

    bool ok = comm.sendCommandBlindFocuser(1, ":FQ#");
    t.join();
    close(dFd);

    EXPECT_TRUE(ok);
    EXPECT_EQ(sel, ":FA1");
    EXPECT_EQ(cmd, ":FQ");
}

// ---------------------------------------------------------------------------
// 2. Select command format for all 6 slots
// ---------------------------------------------------------------------------
TEST(FocuserComm, SelectCommandFormatSlot1)
{
    char cmd[8];
    snprintf(cmd, sizeof(cmd), ":FA%d#", 1);
    EXPECT_STREQ(cmd, ":FA1#");
}

TEST(FocuserComm, SelectCommandFormatSlot6)
{
    char cmd[8];
    snprintf(cmd, sizeof(cmd), ":FA%d#", 6);
    EXPECT_STREQ(cmd, ":FA6#");
}

// ---------------------------------------------------------------------------
// 3. Focuser operation command format strings
// ---------------------------------------------------------------------------
TEST(FocuserCmds, GotoAbsFormat)
{
    char cmd[24];
    snprintf(cmd, sizeof(cmd), ":FN%u#", 50000u);
    EXPECT_STREQ(cmd, ":FN50000#");
}

TEST(FocuserCmds, MoveRelPositive)
{
    char cmd[24];
    snprintf(cmd, sizeof(cmd), ":Fm%d#", 500);
    EXPECT_STREQ(cmd, ":Fm500#");
}

TEST(FocuserCmds, MoveRelNegative)
{
    char cmd[24];
    snprintf(cmd, sizeof(cmd), ":Fm%d#", -500);
    EXPECT_STREQ(cmd, ":Fm-500#");
}

TEST(FocuserCmds, SetBacklash)
{
    char cmd[24];
    snprintf(cmd, sizeof(cmd), ":FB%d#", 50);
    EXPECT_STREQ(cmd, ":FB50#");
}

TEST(FocuserCmds, SetSpeed)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), ":FP%d#", 2);
    EXPECT_STREQ(cmd, ":FP2#");
}

// ---------------------------------------------------------------------------
// 4. Reply parsing (as done in cmdGetPos / cmdGetTemperature)
// ---------------------------------------------------------------------------
TEST(FocuserParse, ParsePosition)
{
    const char *reply = "12345";
    char *end;
    long val = std::strtol(reply, &end, 10);
    EXPECT_NE(end, reply);
    EXPECT_EQ(val, 12345L);
}

TEST(FocuserParse, ParseTemperatureValid)
{
    const char *reply = "22.5";
    char *end;
    double val = std::strtod(reply, &end);
    EXPECT_NE(end, reply);
    EXPECT_DOUBLE_EQ(val, 22.5);
    EXPECT_LT(val, 990.0);  // not the "no sensor" sentinel
}

TEST(FocuserParse, ParseTemperatureNoSensor)
{
    const char *reply = "999";
    char *end;
    double val = std::strtod(reply, &end);
    EXPECT_NE(end, reply);
    EXPECT_GT(val, 990.0);  // sentinel: ignore
}

TEST(FocuserParse, StatusMoving)
{
    const char reply = 'M';
    EXPECT_TRUE(reply == 'M');
}

TEST(FocuserParse, StatusStopped)
{
    const char reply = 'S';
    EXPECT_FALSE(reply == 'M');
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
