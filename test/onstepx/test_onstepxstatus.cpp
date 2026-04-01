/*
    OnStep X INDI Driver — Status parser unit tests (Stage 3)

    OnStepXStatus has zero INDI dependencies — it is pure data transformation.
    Tests run without any device instance or socketpair.
*/

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "OnStepXStatus.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Build a minimal valid :GU# string: flags + "000" (rates/error all zero).
// Usage: buildGU("nNpE") → "nNpE000"
static std::string buildGU(const char *flags)
{
    std::string s = flags;
    s += "000";   // pulseGuideRate=0, guideRate=0, errorCode=0
    return s;
}

// Build a binary :Gu# byte array. All 9 bytes default to 0x80 (valid but
// zero payload). Individual bytes can be overridden via parameters.
static void buildGu(uint8_t out[9],
                    uint8_t b0 = 0x80, uint8_t b1 = 0x80,
                    uint8_t b2 = 0x80, uint8_t b3 = 0x80,
                    uint8_t b4 = 0x80, uint8_t b5 = 0x80,
                    uint8_t b6 = 0x80, uint8_t b7 = 0x80,
                    uint8_t b8 = 0x80)
{
    out[0]=b0; out[1]=b1; out[2]=b2; out[3]=b3; out[4]=b4;
    out[5]=b5; out[6]=b6; out[7]=b7; out[8]=b8;
}

// ---------------------------------------------------------------------------
// :GU# tests
// ---------------------------------------------------------------------------

// Absence of 'n' → tracking = true
TEST(OnStepXStatusTest, test_parseGU_tracking)
{
    MountStatus s;
    // No 'n' in flags — mount is tracking
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("NpE").c_str(), s));
    EXPECT_TRUE(s.tracking);
}

// 'n' present → tracking = false
TEST(OnStepXStatusTest, test_parseGU_not_tracking)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("nNpE").c_str(), s));
    EXPECT_FALSE(s.tracking);
}

// 'P' → parkState = PARKED, tracking stops (implied by firmware but not parsed here)
TEST(OnStepXStatusTest, test_parseGU_parked)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("nNPE").c_str(), s));
    EXPECT_EQ(s.parkState, MountStatus::ParkState::PARKED);
}

// 'I' → parkState = PARKING
TEST(OnStepXStatusTest, test_parseGU_parking)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("nNIE").c_str(), s));
    EXPECT_EQ(s.parkState, MountStatus::ParkState::PARKING);
}

// 'F' → parkState = FAILED
TEST(OnStepXStatusTest, test_parseGU_park_failed)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("nNFE").c_str(), s));
    EXPECT_EQ(s.parkState, MountStatus::ParkState::FAILED);
}

// '~' → pecState = PLAYING
TEST(OnStepXStatusTest, test_parseGU_pec_playing)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("NpE~").c_str(), s));
    EXPECT_EQ(s.pecState, MountStatus::PecState::PLAYING);
}

// 'A' → mountType = ALTAZM
TEST(OnStepXStatusTest, test_parseGU_altazm)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("NpA").c_str(), s));
    EXPECT_EQ(s.mountType, MountStatus::MountType::ALTAZM);
}

// '(' → trackRate = LUNAR
TEST(OnStepXStatusTest, test_parseGU_lunar_rate)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("Np(E").c_str(), s));
    EXPECT_EQ(s.trackRate, MountStatus::TrackRate::LUNAR);
}

// 'k' → trackRate = KING
TEST(OnStepXStatusTest, test_parseGU_king_rate)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("NpkE").c_str(), s));
    EXPECT_EQ(s.trackRate, MountStatus::TrackRate::KING);
}

// 'T' → pierSide = EAST
TEST(OnStepXStatusTest, test_parseGU_pier_east)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("NpET").c_str(), s));
    EXPECT_EQ(s.pierSide, MountStatus::PierSide::EAST);
}

// 'W' → pierSide = WEST
TEST(OnStepXStatusTest, test_parseGU_pier_west)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("NpEW").c_str(), s));
    EXPECT_EQ(s.pierSide, MountStatus::PierSide::WEST);
}

// Error code digit in last position
TEST(OnStepXStatusTest, test_parseGU_error_code)
{
    MountStatus s;
    // Manually construct with error code 5: "NpE" + "005"
    ASSERT_TRUE(OnStepXStatus::parseGU("NpE005", s));
    EXPECT_EQ(s.errorCode, 5);
    EXPECT_EQ(s.pulseGuideRateSelect, 0);
    EXPECT_EQ(s.guideRateSelect, 0);
}

// Empty string → false
TEST(OnStepXStatusTest, test_parseGU_empty)
{
    MountStatus s;
    EXPECT_FALSE(OnStepXStatus::parseGU("", s));
    EXPECT_FALSE(OnStepXStatus::parseGU(nullptr, s));
}

// ---------------------------------------------------------------------------
// :Gu# binary tests
// ---------------------------------------------------------------------------

// Valid 9-byte buffer, all >= 0x80 → true; verify tracking decoded from byte 0.
// Byte 0 = 0x82: bit 1 set (no goto), bit 0 clear (tracking active).
TEST(OnStepXStatusTest, test_parseGu_binary_happy)
{
    uint8_t buf[9];
    // b0=0x82: tracking=true (bit0=0), no goto (bit1=1), unparked (bits2-3=0)
    // b3=0x84: GEM (bits2-3=01 after 0x80 mask → (0x84&0x7F)>>2 = 0x04>>2 = 1 = FORK)
    // Let's keep it simple: all 0x80 = tracking, unparked, GEM, sidereal.
    buildGu(buf);   // all 0x80

    MountStatus s;
    EXPECT_TRUE(OnStepXStatus::parseGu(buf, 9, s));
    // 0x80 & 0x7F = 0x00: bit0=0 → tracking=true
    EXPECT_TRUE(s.tracking);
    // bit1=0 → no-goto bit NOT set → goto IS "active" per the flag
    // (This just means gotoActive=true when bit is 0, which is the default state
    //  from the mount's perspective between operations. This is correct — the
    //  mount is not mid-slew, so gotoActive reflects only current motion.)
    EXPECT_EQ(s.parkState, MountStatus::ParkState::UNPARKED);
    EXPECT_EQ(s.errorCode, 0);
}

// Short buffer (8 bytes) → false
TEST(OnStepXStatusTest, test_parseGu_binary_short)
{
    uint8_t buf[8];
    memset(buf, 0x80, sizeof(buf));
    MountStatus s;
    EXPECT_FALSE(OnStepXStatus::parseGu(buf, 8, s));
}

// Any byte < 0x80 (high-bit not set) → false
TEST(OnStepXStatusTest, test_parseGu_binary_unmasked)
{
    uint8_t buf[9];
    buildGu(buf);
    buf[4] = 0x7F;   // byte 4 has high bit clear — invalid
    MountStatus s;
    EXPECT_FALSE(OnStepXStatus::parseGu(buf, 9, s));
}

// ---------------------------------------------------------------------------
// Additional coverage: tracking-comp and rate parsing
// ---------------------------------------------------------------------------

// 'r' alone → REFRACTION_DUAL
TEST(OnStepXStatusTest, test_parseGU_refraction_dual)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("NprE").c_str(), s));
    EXPECT_EQ(s.trackComp, MountStatus::TrackComp::REFRACTION_DUAL);
}

// 'r'+'s' → REFRACTION_SINGLE
TEST(OnStepXStatusTest, test_parseGU_refraction_single)
{
    MountStatus s;
    ASSERT_TRUE(OnStepXStatus::parseGU(buildGU("NprsE").c_str(), s));
    EXPECT_EQ(s.trackComp, MountStatus::TrackComp::REFRACTION_SINGLE);
}

// ---------------------------------------------------------------------------
// main() — defined here to override indidrivermain.c from libindidriver.so.
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
