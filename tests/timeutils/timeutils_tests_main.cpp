#include <gtest/gtest.h>
#include "libslic3r/Time.hpp"

#include <sstream>
#include <iomanip>
#include <locale>

namespace {

void test_time_fmt(const char *fmt) {
    using namespace Slic3r::Utils;
    time_t t = get_current_time_utc();

    std::string tstr = time2str(t, TimeZone::local, fmt);
    time_t parsedtime = str2time(tstr, TimeZone::local, fmt);

    ASSERT_EQ(t, parsedtime);

    tstr = time2str(t, TimeZone::utc, fmt);
    parsedtime = str2time(tstr, TimeZone::utc, fmt);

    ASSERT_EQ(t, parsedtime);

    parsedtime = str2time("not valid string", TimeZone::local, fmt);
    ASSERT_TRUE(parsedtime < time_t(0));

    parsedtime = str2time("not valid string", TimeZone::utc, fmt);
    ASSERT_TRUE(parsedtime < time_t(0));
}
}

TEST(Timeutils, ISO8601Z) {
    test_time_fmt(Slic3r::Utils::ISO8601Z_TIME_FMT);
}

TEST(Timeutils, Slic3r_TZ_Time_Format) {
    test_time_fmt(Slic3r::Utils::SLICER_TZ_TIME_FMT);
}

TEST(Timeutils, Slic3r_UTC_Time_Format) {
    test_time_fmt(Slic3r::Utils::SLICER_UTC_TIME_FMT);
}

// There is no working std::strptime or std::get_time in MSVC currently, so
// no way to run the back and forth conversion test without a third party impl.
#ifndef _MSC_VER
TEST(Timeutils, Locale_Time_Format) {
    std::locale::global(std::locale(setlocale(LC_ALL, "")));
    test_time_fmt(Slic3r::Utils::LOCALE_TIME_FMT);
    
    std::locale::global(std::locale(setlocale(LC_ALL, "zh_TW")));
    test_time_fmt(Slic3r::Utils::LOCALE_TIME_FMT);
}
#endif

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
