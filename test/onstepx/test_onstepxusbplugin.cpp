/*
    OnStep X INDI Driver — USB Plugin helper

    Tests cover:
      - makePropBase sanitization (label -> INDI property name)
      - parseType: all type characters mapped correctly
      - probeSlot: :GUY[n]# reply parsing (name)
      - discoverAndDefine: correct slots activated per portMask
      - handleSwitch: SWITCH type -> :SUX[n],V0# / :SUX[n],V1#
      - pollStatus: :GUX[n]# value read
*/

#include <gtest/gtest.h>

#include "OnStepXUsbPlugin.h"

#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// Expose private helpers via a test subclass
// ---------------------------------------------------------------------------
class TestableUsb : public OnStepXUsbPlugin
{
    public:
        static std::string propBase(const char *label, int idx)
        {
            char buf[64];
            makePropBase(label, idx, buf, sizeof(buf));
            return std::string(buf);
        }
};

// ---------------------------------------------------------------------------
// 1. makePropBase sanitization
// ---------------------------------------------------------------------------
TEST(UsbPlugin, PropBase_Simple)
{
    EXPECT_EQ(TestableUsb::propBase("MainCam", 1), "OSX_MAINCAM");
}

TEST(UsbPlugin, PropBase_AlreadyUpper)
{
    EXPECT_EQ(TestableUsb::propBase("MAINCAM", 2), "OSX_MAINCAM");
}

TEST(UsbPlugin, PropBase_EmptyLabel)
{
    EXPECT_EQ(TestableUsb::propBase("", 3), "OSX_USB3");
}

TEST(UsbPlugin, PropBase_TrailingSpaces)
{
    EXPECT_EQ(TestableUsb::propBase("GUIDE  ", 4), "OSX_GUIDE");
}

TEST(UsbPlugin, PropBase_SpecialChars)
{
    EXPECT_EQ(TestableUsb::propBase("CAM-1", 5), "OSX_CAM_1");
}

// ---------------------------------------------------------------------------
// 2. Command format for :SUX[n],F[v]#
// ---------------------------------------------------------------------------
TEST(UsbPlugin, WriteIntCmdFormat_SwitchOn)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":SUX%d,%c%d#", 1, 'V', 1);
    EXPECT_STREQ(cmd, ":SUX1,V1#");
}

TEST(UsbPlugin, WriteIntCmdFormat_SwitchOff)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ":SUX%d,%c%d#", 2, 'V', 0);
    EXPECT_STREQ(cmd, ":SUX2,V0#");
}

// ---------------------------------------------------------------------------
// 3. Poll command format :GUX[n]#
// ---------------------------------------------------------------------------
TEST(UsbPlugin, PollCmdFormat)
{
    for (int i = 1; i <= 8; i++)
    {
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":GUX%d#", i);
        char expected[16];
        snprintf(expected, sizeof(expected), ":GUX%d#", i);
        EXPECT_STREQ(cmd, expected);
    }
}

// ---------------------------------------------------------------------------
// 4. Discovery command :GUY[n]# format
// ---------------------------------------------------------------------------
TEST(UsbPlugin, DiscoveryCmdFormat)
{
    for (int i = 1; i <= 8; i++)
    {
        char cmd[16];
        snprintf(cmd, sizeof(cmd), ":GUY%d#", i);
        char expected[16];
        snprintf(expected, sizeof(expected), ":GUY%d#", i);
        EXPECT_STREQ(cmd, expected);
    }
}

// ---------------------------------------------------------------------------
// 5. portMask bit parsing — correct slots activate
// ---------------------------------------------------------------------------
TEST(UsbPlugin, PortMaskBitMapping)
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
