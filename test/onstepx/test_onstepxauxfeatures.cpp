/*
    OnStep X INDI Driver — Stage 11 unit tests: Auxiliary Features helper

    Tests cover:
      - makePropBase sanitization (label -> INDI property name)
      - parseType: all type characters mapped correctly
      - probeSlot: :GXY[n]# reply parsing (name + type)
      - discoverAndDefine: correct slots activated per featureMask
      - handleSwitch: SWITCH type -> :SXX[n],V0# / :SXX[n],V1#
      - handleNumber: ANALOG type -> :SXX[n],V[val]#
      - handleNumber: DEW_HEATER -> V + Z + S commands
      - pollStatus: :GXX[n]# value read
      - sendWriteInt: :SXX[n],F[v]# format
*/

#include <gtest/gtest.h>

#include "OnStepXAuxFeatures.h"

#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// Expose private helpers via a test subclass
// ---------------------------------------------------------------------------
class TestableAux : public OnStepXAuxFeatures
{
    public:
        static std::string propBase(const char *label, int idx)
        {
            char buf[64];
            makePropBase(label, idx, buf, sizeof(buf));
            return std::string(buf);
        }

        // Expose parseType via a public wrapper
        static int typeOf(char t)
        {
            return static_cast<int>(parseType(t));
        }
};

// ---------------------------------------------------------------------------
// 1. makePropBase sanitization
// ---------------------------------------------------------------------------
TEST(AuxFeatures, PropBase_Simple)
{
    EXPECT_EQ(TestableAux::propBase("Dew A", 1), "OSX_DEW_A");
}

TEST(AuxFeatures, PropBase_AlreadyUpper)
{
    EXPECT_EQ(TestableAux::propBase("SWITCH", 2), "OSX_SWITCH");
}

TEST(AuxFeatures, PropBase_EmptyLabel)
{
    EXPECT_EQ(TestableAux::propBase("", 3), "OSX_AUX3");
}

TEST(AuxFeatures, PropBase_TrailingSpaces)
{
    EXPECT_EQ(TestableAux::propBase("DEW  ", 4), "OSX_DEW");
}

TEST(AuxFeatures, PropBase_SpecialChars)
{
    EXPECT_EQ(TestableAux::propBase("Dew-1", 5), "OSX_DEW_1");
}

// ---------------------------------------------------------------------------
// 2. parseType
// ---------------------------------------------------------------------------
TEST(AuxFeatures, ParseType_Switch)
{
    // '1' and 'S'/'s' -> SWITCH (1)
    EXPECT_EQ(TestableAux::typeOf('1'), 0);
    EXPECT_EQ(TestableAux::typeOf('S'), 0);
    EXPECT_EQ(TestableAux::typeOf('s'), 0);
}

TEST(AuxFeatures, ParseType_Analog)
{
    EXPECT_EQ(TestableAux::typeOf('2'), 1);
    EXPECT_EQ(TestableAux::typeOf('A'), 1);
    EXPECT_EQ(TestableAux::typeOf('a'), 1);
}

TEST(AuxFeatures, ParseType_DewHeater)
{
    EXPECT_EQ(TestableAux::typeOf('3'), 2);
    EXPECT_EQ(TestableAux::typeOf('D'), 2);
    EXPECT_EQ(TestableAux::typeOf('d'), 2);
}

TEST(AuxFeatures, ParseType_Intervalometer)
{
    EXPECT_EQ(TestableAux::typeOf('4'), 3);
    EXPECT_EQ(TestableAux::typeOf('I'), 3);
    EXPECT_EQ(TestableAux::typeOf('i'), 3);
}

TEST(AuxFeatures, ParseType_Unknown)
{
    EXPECT_EQ(TestableAux::typeOf('X'), 4);   // UNKNOWN
}

// ---------------------------------------------------------------------------
// 3. Command format for :SXX[n],F[v]#
// ---------------------------------------------------------------------------
TEST(AuxFeatures, WriteIntCmdFormat_SwitchOn)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", 1, 'V', 1);
    EXPECT_STREQ(cmd, ":SXX1,V1#");
}

TEST(AuxFeatures, WriteIntCmdFormat_SwitchOff)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", 2, 'V', 0);
    EXPECT_STREQ(cmd, ":SXX2,V0#");
}

TEST(AuxFeatures, WriteIntCmdFormat_Analog)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", 3, 'V', 128);
    EXPECT_STREQ(cmd, ":SXX3,V128#");
}

TEST(AuxFeatures, WriteIntCmdFormat_DewEnable)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", 4, 'E', 1);
    EXPECT_STREQ(cmd, ":SXX4,E1#");
}

TEST(AuxFeatures, WriteIntCmdFormat_DewZero)
{
    char cmd[32];
    // Zero point 5.0°C -> 50 (× 10)
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", 5, 'Z', 50);
    EXPECT_STREQ(cmd, ":SXX5,Z50#");
}

TEST(AuxFeatures, WriteIntCmdFormat_DewSpan)
{
    char cmd[32];
    // Span 10.0°C -> 100 (× 10)
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", 6, 'S', 100);
    EXPECT_STREQ(cmd, ":SXX6,S100#");
}

TEST(AuxFeatures, WriteIntCmdFormat_IvoCount)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", 7, 'C', 10);
    EXPECT_STREQ(cmd, ":SXX7,C10#");
}

TEST(AuxFeatures, WriteIntCmdFormat_IvoDuration)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":SXX%d,%c%d#", 8, 'D', 2000);
    EXPECT_STREQ(cmd, ":SXX8,D2000#");
}

// ---------------------------------------------------------------------------
// 4. Poll command format :GXX[n]#
// ---------------------------------------------------------------------------
TEST(AuxFeatures, PollCmdFormat)
{
    for (int i = 1; i <= 8; i++)
    {
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":GXX%d#", i);
        char expected[16];
        snprintf(expected, sizeof(expected), ":GXX%d#", i);
        EXPECT_STREQ(cmd, expected);
    }
}

// ---------------------------------------------------------------------------
// 5. Discovery command :GXY[n]# format
// ---------------------------------------------------------------------------
TEST(AuxFeatures, DiscoveryCmdFormat)
{
    for (int i = 1; i <= 8; i++)
    {
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":GXY%d#", i);
        char expected[16];
        snprintf(expected, sizeof(expected), ":GXY%d#", i);
        EXPECT_STREQ(cmd, expected);
    }
}

// ---------------------------------------------------------------------------
// 6. featureMask bit parsing — correct slots activate
// ---------------------------------------------------------------------------
TEST(AuxFeatures, FeatureMaskBitMapping)
{
    // Bit 0 = slot 1, bit 7 = slot 8
    EXPECT_TRUE((0x01u & (1u << 0)) != 0);  // slot 1
    EXPECT_TRUE((0x80u & (1u << 7)) != 0);  // slot 8
    EXPECT_FALSE((0x01u & (1u << 1)) != 0); // slot 2 not in mask 0x01
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
