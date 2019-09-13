#include <gtest/gtest.h>
#include "libslic3r/Time.hpp"

#include <sstream>
#include <iomanip>
#include <locale>

//#ifdef _MSC_VER
//// Even in VS2019, std::get_time is buggy and can not parse ISO8601Z_TIME_FMT
//// nor LOCALE_TIME_FMT
//// until it gets corrected, here is the original parsing code with sscanf
//time_t parse_time_ISO8601Z(const std::string &sdate)
//{
//    int y, M, d, h, m, s;
//    if (sscanf(sdate.c_str(), "%04d%02d%02dT%02d%02d%02dZ", &y, &M, &d, &h, &m, &s) != 6)
//        return time_t(-1);
//    struct tm tms;
//    tms.tm_year = y - 1900;  // Year since 1900
//    tms.tm_mon  = M - 1;     // 0-11
//    tms.tm_mday = d;         // 1-31
//    tms.tm_hour = h;         // 0-23
//    tms.tm_min  = m;         // 0-59
//    tms.tm_sec  = s;         // 0-61 (0-60 in C++11)
//#ifdef WIN32
//    return _mkgmtime(&tms);
//#else /* WIN32 */
//    return timegm(&tms);
//#endif /* WIN32 */
//}
//#endif

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

// There is no working std::strptime or std::get_time MSVC currently, so
// no way to run the back and forth conversion test.
#ifndef _MSC_VER
TEST(Timeutils, Locale_Time_Format) {
    std::locale::global(std::locale(setlocale(LC_ALL, "")));
    test_time_fmt(Slic3r::Utils::LOCALE_TIME_FMT);
}
#endif

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
