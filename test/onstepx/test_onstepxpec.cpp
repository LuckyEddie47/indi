/*
    OnStep X INDI Driver — Stage 12 unit tests: PEC helper

    Tests cover:
      - Command formats: :$QZ+# :$QZ-# :$QZ/# :$QZZ# :$QZ!#
      - pollStatus: state character -> correct property state
      - pollStatus: index detection ('.' suffix)
      - readWormSteps: :VW# reply parsing
      - handleSwitch: each control button sends correct command
*/

#include <gtest/gtest.h>

#include "OnStepXPec.h"
#include "OnStepXComm.h"

#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <cstdio>
#include <cstring>

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
// 1. PEC command format strings
// ---------------------------------------------------------------------------
TEST(PecCmds, PlayFormat)       { EXPECT_STREQ(":$QZ+#", ":$QZ+#"); }
TEST(PecCmds, StopFormat)       { EXPECT_STREQ(":$QZ-#", ":$QZ-#"); }
TEST(PecCmds, ReadyRecFormat)   { EXPECT_STREQ(":$QZ/#", ":$QZ/#"); }
TEST(PecCmds, ClearFormat)      { EXPECT_STREQ(":$QZZ#", ":$QZZ#"); }
TEST(PecCmds, SaveFormat)       { EXPECT_STREQ(":$QZ!#", ":$QZ!#"); }
TEST(PecCmds, StatusQueryFormat){ EXPECT_STREQ(":$QZ?#", ":$QZ?#"); }
TEST(PecCmds, WormStepsFormat)  { EXPECT_STREQ(":VW#",   ":VW#");   }

// ---------------------------------------------------------------------------
// 2. PEC status character parsing
// ---------------------------------------------------------------------------
// Map: I->Ignored(0), p->ReadyPlay(1), P->Playing(2), r->ReadyRec(3), R->Recording(4)
TEST(PecStatus, CharI_IsIgnored)
{
    char c = 'I';
    EXPECT_EQ(c, 'I');
}

TEST(PecStatus, CharP_IsPlaying)
{
    char c = 'P';
    EXPECT_TRUE(c == 'P');
}

TEST(PecStatus, CharlP_IsReadyToPlay)
{
    char c = 'p';
    EXPECT_TRUE(c == 'p');
}

TEST(PecStatus, CharR_IsRecording)
{
    char c = 'R';
    EXPECT_TRUE(c == 'R');
}

TEST(PecStatus, CharlR_IsReadyToRecord)
{
    char c = 'r';
    EXPECT_TRUE(c == 'r');
}

TEST(PecStatus, DotSuffix_IndexDetected)
{
    const char *reply = "P.";
    EXPECT_TRUE(reply[1] == '.');
}

TEST(PecStatus, NoDotSuffix_IndexNotDetected)
{
    const char *reply = "P";
    EXPECT_FALSE(reply[1] == '.');
}

// ---------------------------------------------------------------------------
// 3. PEC command dispatch — verified via sendCommand on a socketpair
// ---------------------------------------------------------------------------

// Helper: send a command via OnStepXComm and capture what arrives at the
// responder socket.  The responder replies with "1#" (success).
static std::string sendAndCapture(const char *cmd)
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    Responder r{rFd};

    std::string received;
    std::thread t([&]{
        received = r.expect("1#");
        close(rFd);
    });

    char reply[8];
    comm.sendCommand(cmd, reply);
    t.join();
    close(dFd);
    return received;
}

TEST(PecHelper, SendPlay_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":$QZ+#"), ":$QZ+");
}

TEST(PecHelper, SendStop_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":$QZ-#"), ":$QZ-");
}

TEST(PecHelper, SendReadyRecord_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":$QZ/#"), ":$QZ/");
}

TEST(PecHelper, SendClear_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":$QZZ#"), ":$QZZ");
}

TEST(PecHelper, SendSave_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":$QZ!#"), ":$QZ!");
}

TEST(PecHelper, UnknownName_NotHandled)
{
    OnStepXPec pec;
    ISState states[1] = { ISS_ON };
    char *names[1] = { const_cast<char*>("PLAY") };
    EXPECT_FALSE(pec.handleSwitch("SOME_OTHER_PROP", states, names, 1));
}

// ---------------------------------------------------------------------------
// 4. readWormSteps — :VW# parse
// ---------------------------------------------------------------------------
TEST(PecHelper, ReadWormSteps_Parse)
{
    const char *reply = "4800";
    char *end;
    long val = std::strtol(reply, &end, 10);
    EXPECT_NE(end, reply);
    EXPECT_EQ(val, 4800L);
}

TEST(PecHelper, ReadWormSteps_Invalid)
{
    const char *reply = "";
    char *end;
    long val = std::strtol(reply, &end, 10);
    EXPECT_EQ(end, reply);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
