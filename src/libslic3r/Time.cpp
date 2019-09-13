#include "Time.hpp"

#include <iomanip>
#include <codecvt>
#include <sstream>
#include <chrono>
#include <cassert>
#include <ctime>
#include <cstdio>
#include <clocale>
#include <map>

#include "libslic3r/Utils.hpp"

namespace Slic3r {
namespace Utils {

namespace {

inline std::string to_utf8(const std::wstring &wstr)
{
    std::wstring_convert<std::codecvt_utf8<wchar_t>> myconv;
    return myconv.to_bytes(wstr);
}

inline std::wstring from_utf8(const std::string &str)
{
    std::wstring_convert<std::codecvt_utf8<wchar_t>> myconv;
    return myconv.from_bytes(str);
}

inline std::string to_utf8(const std::string &str) { return str; }

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

// VS2019 implementation satisfies SLICER_TIME_FMT but fails with ISO8601Z_TIME_FMT
// and LOCALE_TIME_FMT. VS2019 does not have std::strptime: can't use the fallback...
#ifdef _MSC_VER
static const std::map<std::string, std::string> fallback_map = {
    {SLICER_TZ_TIME_FMT, "%04d-%02d-%02d at %02d:%02d:%02d"},
    {SLICER_UTC_TIME_FMT, "%04d-%02d-%02d at %02d:%02d:%02d"},
    {ISO8601Z_TIME_FMT, "%04d%02d%02dT%02d%02d%02dZ"}
};

bool strptime(const char *str, const char *const fmt, std::tm *tms)
{
    int y, M, d, h, m, s;
    if (sscanf(str, fmt, &y, &M, &d, &h, &m, &s) != 6)
        return false;
    
    tms->tm_year = y - 1900;  // Year since 1900
    tms->tm_mon  = M - 1;     // 0-11
    tms->tm_mday = d;         // 1-31
    tms->tm_hour = h;         // 0-23
    tms->tm_min  = m;         // 0-59
    tms->tm_sec  = s;         // 0-61 (0-60 in C++11)
    
    return true;
}
#endif

template<class TChar>
std::basic_istream<TChar> &operator>>(std::basic_istream<TChar> &stream,
                                      GetTimeReturnT<TChar> &&gt)
{
#ifdef _MSC_VER
    auto it = fallback_map.find(to_utf8(fmt));
    if (it == fallback_map.end()) {
        stream >> std::get_time(gt.tms, gt.fmt);
    } else {
#endif
    std::basic_string<TChar> str;
    std::getline(stream, str);
    std::string utf8str = to_utf8(str);
    strptime(utf8str.c_str(), gt.fmt, gt.tms);
#ifdef _MSC_VER        
    }
#endif

    return stream;
}

template<class TChar>
inline GetTimeReturnT<TChar> _get_time(std::tm *tms, const TChar *fmt)
{
    return {tms, fmt};
}

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

std::string process_format(const char *fmt, TimeZone zone)
{
    std::string fmtstr(fmt);

    if (fmtstr == SLICER_UTC_TIME_FMT && zone == TimeZone::utc)
        fmtstr += " UTC";

    return fmtstr;
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

static std::string tm2str(const std::tm *tms, const char *fmt)
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
    std::string fmtstr = process_format(fmt, zone);

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
    std::string fmtstr = process_format(_fmt, zone);
#ifdef _MSC_VER
    std::wstring wstr = from_utf8(str);
    std::wstringstream ss(wstr);
    std::wstring wfmt = from_utf8(fmtstr);
    auto fmt = wfmt.c_str();
#else
    auto fmt = fmtstr.c_str();
    std::stringstream ss(str);
#endif
    ss.imbue(std::locale(setlocale(LC_ALL, nullptr)));
    return str2time(ss, zone, fmt);
}

}; // namespace Utils
}; // namespace Slic3r
