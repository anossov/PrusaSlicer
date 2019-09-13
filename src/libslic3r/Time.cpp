#include "Time.hpp"

#include <iomanip>
#include <sstream>
#include <chrono>
#include <cassert>
#include <ctime>
#include <cstdio>

#include <clocale>

#include "libslic3r/Utils.hpp"

//#include <boost/date_time/local_time/local_time.hpp>

#ifdef WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
    #undef WIN32_LEAN_AND_MEAN
#endif /* WIN32 */

namespace Slic3r {
namespace Utils {

namespace {

// FIXME: Implementations with the cpp11 put_time and get_time either not compile
// or do not pass the tests on the build server

template<class TChar, class Ttm>
struct GetPutTimeReturnT {
    Ttm *tms;
    const TChar *fmt;
    GetPutTimeReturnT(Ttm *_tms, const TChar *_fmt): tms(_tms), fmt(_fmt) {}
};

template<class TChar> using PutTimeReturnT = GetPutTimeReturnT<TChar, const std::tm>;
template<class TChar> using GetTimeReturnT = GetPutTimeReturnT<TChar, std::tm>;

template<class TChar>
inline size_t _strftime(TChar *out, size_t, const TChar *, const std::tm* tms);

template<>
inline size_t _strftime(char *out, size_t s, const char * in, const std::tm* tms)
{
    return std::strftime(out, s, in, tms);
}

template<>
inline size_t _strftime(wchar_t *out, size_t s, const wchar_t * in, const std::tm* tms)
{
    return std::wcsftime(out, s, in, tms);
}

template<class TChar>
std::basic_ostream<TChar> &operator<<(std::basic_ostream<TChar> &stream,
                                      PutTimeReturnT<TChar> &&   pt)
{
    static const constexpr int MAX_CHARS = 200;
    TChar _out[MAX_CHARS];
    _strftime(_out, MAX_CHARS, pt.fmt, pt.tms);
    stream << _out;
    return stream;
}

template<class TChar>
inline PutTimeReturnT<TChar> _put_time(const std::tm *tms, const TChar *fmt)
{
    return {tms, fmt};
}

#ifdef _MSC_VER
// VS2019 implementation satisfies SLICER_TIME_FMT but fails with ISO8601Z_TIME_FMT
// and LOCALE_TIME_FMT. VS2019 does not have std::strptime: can't use the fallback...
template<class TChar>
inline auto _get_time(std::tm *t, const TChar *f) -> decltype (std::get_time(t, f))
{
    return std::get_time(t, f);
}
#else
template<class TChar>
std::basic_istream<TChar> &operator>>(std::basic_istream<TChar> &stream,
                                      GetPutTimeReturnT<TChar> &&gt)
{
    std::basic_string<TChar> str;
    std::getline(stream, str);
    std::string utf8str = to_utf8(str);
    strptime(utf8str.c_str(), gt.fmt, gt.tms);
    return stream;
}

template<class TChar>
inline GetPutTimeReturnT<TChar> _get_time(std::tm *tms, const TChar *fmt)
{
    return {tms, fmt};
}
#endif

// Platform independent versions of gmtime and localtime. Completely thread
// safe only on Linux. MSVC gtime_s and localtime_s sets global errno thus not
// thread safe.
struct std::tm * _gmtime_r(const time_t *timep, struct tm *result)
{
    assert(timep != nullptr && result != nullptr);
#ifdef WIN32
    time_t t = *timep;
    gmtime_s(result, &t);
    return result;
#else
    return gmtime_r(timep, result);
#endif
}

struct std::tm * _localtime_r(const time_t *timep, struct tm *result)
{
    assert(timep != nullptr && result != nullptr);
#ifdef WIN32
    // Converts a time_t time value to a tm structure, and corrects for the
    // local time zone.
    time_t t = *timep;
    localtime_s(result, &t);
    return result;
#else
    return localtime_r(timep, result);
#endif
}

time_t _mktime(const struct std::tm *tms)
{
    assert(tms != nullptr);
    std::tm _tms = *tms;
    return mktime(&_tms);
}

time_t _timegm(const struct std::tm *tms)
{
    std::tm _tms = *tms;
#ifdef WIN32
    return _mkgmtime(&_tms);
#else /* WIN32 */
    return timegm(&_tms);
#endif /* WIN32 */
}

} // namespace

time_t get_current_time_utc()
{
    using clk = std::chrono::system_clock;
    return clk::to_time_t(clk::now());
}

template<class TChar>
std::string _tm2str(const std::tm *tms, const TChar *fmt)
{
    std::basic_stringstream<TChar> ss;
    ss.imbue(std::locale(setlocale(LC_ALL, nullptr)));
    ss << _put_time(tms, fmt);
    return to_utf8(ss.str());
}

std::string tm2str(const std::tm *tms, const char *fmt)
{
#ifdef _MSC_VER
    return _tm2str(tms, from_utf8(fmt).c_str());
#else
    return _tm2str(tms, fmt);
#endif
}

std::string time2str(const time_t &t, TimeZone zone, const char *fmt)
{
    std::string ret;
    std::tm tms = {};
    tms.tm_isdst = -1;
    switch (zone) {
    case TimeZone::local: ret = tm2str(_localtime_r(&t, &tms), fmt); break;
    case TimeZone::utc:   ret = tm2str(_gmtime_r(&t, &tms), fmt); break;
    }

    return ret;
}

template<class TChar>
time_t str2time(std::basic_istream<TChar> &stream, TimeZone zone, const TChar *fmt)
{
    std::tm tms = {};
    tms.tm_isdst = -1;
    stream >> _get_time(&tms, fmt);
    time_t ret = time_t(-1);

    switch (zone) {
    case TimeZone::local: ret = _mktime(&tms); break;
    case TimeZone::utc:   ret = _timegm(&tms); break;
    }

    return ret;
}

time_t str2time(const std::string &str, TimeZone zone, const char *_fmt)
{
#ifdef _MSC_VER
    std::wstring wstr = from_utf8(str);
    std::wstringstream ss(wstr);
    std::wstring wfmt = from_utf8(_fmt);
    auto fmt = wfmt.c_str();
#else
    auto fmt = _fmt;
    std::stringstream ss(str);
#endif
    ss.imbue(std::locale(setlocale(LC_ALL, nullptr)));
    return str2time(ss, zone, fmt);
}

#ifdef _MSC_VER
// Even in VS2019, std::get_time is buggy and can not parse ISO8601Z_TIME_FMT
// nor LOCALE_TIME_FMT
// until it gets corrected, here is the original parsing code with sscanf
time_t parse_time_ISO8601Z(const std::string &sdate)
{
    int y, M, d, h, m, s;
    if (sscanf(sdate.c_str(), "%04d%02d%02dT%02d%02d%02dZ", &y, &M, &d, &h, &m, &s) != 6)
        return time_t(-1);
    struct tm tms;
    tms.tm_year = y - 1900;  // Year since 1900
    tms.tm_mon  = M - 1;     // 0-11
    tms.tm_mday = d;         // 1-31
    tms.tm_hour = h;         // 0-23
    tms.tm_min  = m;         // 0-59
    tms.tm_sec  = s;         // 0-61 (0-60 in C++11)
#ifdef WIN32
    return _mkgmtime(&tms);
#else /* WIN32 */
    return timegm(&tms);
#endif /* WIN32 */
}
#endif

}; // namespace Utils
}; // namespace Slic3r
