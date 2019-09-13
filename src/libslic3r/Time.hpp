#ifndef slic3r_Utils_Time_hpp_
#define slic3r_Utils_Time_hpp_

#include <string>
#include <ctime>

namespace Slic3r {
namespace Utils {

// Should be thread safe.
time_t get_current_time_utc();

// "YYYY-MM-DD at HH:MM::SS TZ"
// TZ is the time zone and can be e.g. GMT, CEST or similar code
// On Windows, depending on the registry settings, it can spell out the whole
// time zone string.
const constexpr char *const SLICER_TZ_TIME_FMT = "%Y-%m-%d at %T %Z";

// SLICER_TZ_TIME_FMT without the time zone codes. If TimeZone::utc is used
// with the conversion functions, it will append the UTC letters to the end.
const constexpr char *const SLICER_UTC_TIME_FMT = "%Y-%m-%d at %T";

// ISO8601Z representation of time, without time zone info
const constexpr char *const ISO8601Z_TIME_FMT = "%Y%m%dT%H%M%SZ";

// FIXME MSVC std::get_time fails with this format...
// The local, locale dependent time formatting.
// If localized time formatting is needed, might be better to use the GUI
// toolkit functions. If used with time2str the currently set C locale
// (set with setlocale()) determines the output format.
const constexpr char *const LOCALE_TIME_FMT = "%x, %B %X";

enum class TimeZone { local, utc };

// time_t to string functions (return date in utf8)...

std::string time2str(const time_t &t, TimeZone zone, const char *fmt = SLICER_TZ_TIME_FMT);

inline std::string current_time2str(TimeZone zone, const char *fmt = SLICER_TZ_TIME_FMT)
{
    return time2str(get_current_time_utc(), zone, fmt);
}

// Current time in the local time zone
inline std::string current_local_time2str(const char * fmt = SLICER_UTC_TIME_FMT)
{
    return current_time2str(TimeZone::local, fmt);
}

inline std::string current_utc_time2str(const char * fmt = SLICER_UTC_TIME_FMT)
{
    return current_time2str(TimeZone::utc, fmt);
}

// string to time_t functions...

// This function is unsafe until std::get_time and std::put_time will be
// available and bug free on all platforms.
// time_t str2time(std::istream &, TimeZone zone, const char *fmt);
time_t str2time(const std::string &str, TimeZone zone, const char *fmt);

// /////////////////////////////////////////////////////////////////////////////
// Utilities to convert an UTC time_t to/from an ISO8601 time format,
// useful for putting timestamps into file and directory names.
// Returns (time_t)-1 on error.

// Use these two functions to convert safely to and from the ISO8601 format on
// all platforms

inline std::string format_time_ISO8601Z(time_t time)
{
    return time2str(time, TimeZone::utc, ISO8601Z_TIME_FMT);
}

inline time_t parse_time_ISO8601Z(const std::string &s)
{
    return str2time(s, TimeZone::utc, ISO8601Z_TIME_FMT);
}
// /////////////////////////////////////////////////////////////////////////////

} // namespace Utils
} // namespace Slic3r

#endif /* slic3r_Utils_Time_hpp_ */
