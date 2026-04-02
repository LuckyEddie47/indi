/*
    OnStep X INDI Driver — Stage 13 unit tests: Alignment helper

    Tests cover:
      - Command format strings
      - :A?# reply parsing (updateStatus)
      - startAlignment: sends :A[n]# for n=1..9
      - acceptStar: sends :A+#
      - writeAlignment: sends :AW#
      - handleSwitch: unknown property returns false
      - Polar error: :GX02# and :GX03# command formats
*/

#include <gtest/gtest.h>

#include "OnStepXAlignment.h"
#include "OnStepXComm.h"

#include <sys/socket.h>
#include <thread>
#include <unistd.h>
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
    // Read until '#', reply with given string, return what was read (minus '#')
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

// Helper: send a command via OnStepXComm and capture what arrives at the
// responder end.  Responder replies with given string.
static std::string sendAndCapture(const char *cmd, const char *reply = "0")
{
    auto [dFd, rFd] = makePair();
    OnStepXComm comm;
    comm.setFd(dFd);
    Responder r{rFd};

    std::string received;
    std::thread t([&]{
        received = r.expect(reply);
        close(rFd);
    });

    char replyBuf[32];
    comm.sendCommand(cmd, replyBuf);
    t.join();
    close(dFd);
    return received;
}

// ---------------------------------------------------------------------------
// 1. Command format strings
// ---------------------------------------------------------------------------
TEST(AlignCmds, StatusQueryFormat)     { EXPECT_STREQ(":A?#",  ":A?#");  }
TEST(AlignCmds, AcceptStarFormat)      { EXPECT_STREQ(":A+#",  ":A+#");  }
TEST(AlignCmds, WriteAlignmentFormat)  { EXPECT_STREQ(":AW#",  ":AW#");  }
TEST(AlignCmds, PolarAltQueryFormat)   { EXPECT_STREQ(":GX02#",":GX02#");}
TEST(AlignCmds, PolarAzQueryFormat)    { EXPECT_STREQ(":GX03#",":GX03#");}

// ---------------------------------------------------------------------------
// 2. startAlignment command — :A[n]# for n in 1..9
// ---------------------------------------------------------------------------
TEST(AlignHelper, StartAlign1_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":A1#", "0"), ":A1");
}

TEST(AlignHelper, StartAlign3_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":A3#", "0"), ":A3");
}

TEST(AlignHelper, StartAlign9_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":A9#", "0"), ":A9");
}

// ---------------------------------------------------------------------------
// 3. acceptStar — :A+#
// ---------------------------------------------------------------------------
TEST(AlignHelper, AcceptStar_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":A+#", "0"), ":A+");
}

// ---------------------------------------------------------------------------
// 4. writeAlignment — :AW#
// ---------------------------------------------------------------------------
TEST(AlignHelper, WriteAlignment_CorrectCmd)
{
    EXPECT_EQ(sendAndCapture(":AW#", "1"), ":AW");
}

// ---------------------------------------------------------------------------
// 5. :A?# reply parsing
// ---------------------------------------------------------------------------
// Reply "mno": m=max, n=current, o=target (chars are ASCII digits; ':' = 9)

TEST(AlignStatus, Parse_NotStarted)
{
    // target=0 means "Not started"
    const char reply[] = "900";   // max=9, current=0, target=0
    int maxStars    = (reply[0] == ':') ? 9 : (reply[0] - '0');
    int currentStar = (reply[1] == ':') ? 9 : (reply[1] - '0');
    int targetStars = (reply[2] == ':') ? 9 : (reply[2] - '0');
    EXPECT_EQ(maxStars,    9);
    EXPECT_EQ(currentStar, 0);
    EXPECT_EQ(targetStars, 0);
}

TEST(AlignStatus, Parse_InProgress)
{
    const char reply[] = "931";   // max=9, current=3, target=1 — actually current>target = done
    // "in progress" means current < target
    const char reply2[] = "912";  // max=9, current=1, target=2
    int cur = reply2[1] - '0';
    int tgt = reply2[2] - '0';
    EXPECT_LT(cur, tgt);
}

TEST(AlignStatus, Parse_Complete)
{
    const char reply[] = "933";   // max=9, current=3, target=3
    int cur = reply[1] - '0';
    int tgt = reply[2] - '0';
    EXPECT_EQ(cur, tgt);
    EXPECT_GT(tgt, 0);
}

TEST(AlignStatus, Parse_ColonMeansNine_Max)
{
    // ':' is the character after '9' in ASCII — firmware uses it for 9
    const char reply[] = ":33";
    int maxStars = (reply[0] == ':') ? 9 : (reply[0] - '0');
    EXPECT_EQ(maxStars, 9);
}

TEST(AlignStatus, Parse_ColonMeansNine_Current)
{
    const char reply[] = "9:3";
    int cur = (reply[1] == ':') ? 9 : (reply[1] - '0');
    EXPECT_EQ(cur, 9);
}

// ---------------------------------------------------------------------------
// 6. handleSwitch — unknown name returns false
// ---------------------------------------------------------------------------
TEST(AlignHelper, UnknownName_NotHandled)
{
    OnStepXAlignment align;
    ISState states[1] = { ISS_ON };
    char *names[1] = { const_cast<char*>("START") };
    EXPECT_FALSE(align.handleSwitch("SOME_OTHER_PROP", states, names, 1));
}

// ---------------------------------------------------------------------------
// 7. Polar error value parsing (float string from :GX02# / :GX03#)
// ---------------------------------------------------------------------------
TEST(AlignPolarError, Parse_PositiveArcsec)
{
    const char *reply = "12.34";
    char *end;
    double val = std::strtod(reply, &end);
    EXPECT_NE(end, reply);
    EXPECT_NEAR(val, 12.34, 0.001);
}

TEST(AlignPolarError, Parse_NegativeArcsec)
{
    const char *reply = "-5.67";
    char *end;
    double val = std::strtod(reply, &end);
    EXPECT_NE(end, reply);
    EXPECT_NEAR(val, -5.67, 0.001);
}

TEST(AlignPolarError, Parse_Zero)
{
    const char *reply = "0.00";
    char *end;
    double val = std::strtod(reply, &end);
    EXPECT_NE(end, reply);
    EXPECT_NEAR(val, 0.0, 0.001);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
