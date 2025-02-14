using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace valhalla.baldr
{
    public static class DateTime
    {
        private static readonly DateTime pivotDate = new DateTime(2015, 1, 1);

        public static DateTime GetFormattedDate(string date, bool canThrow = false)
        {
            DateTime result;
            if (DateTime.TryParseExact(date, "yyyy-MM-ddTHH:mm", CultureInfo.InvariantCulture, DateTimeStyles.None, out result) ||
                DateTime.TryParseExact(date, "yyyy-MM-dd", CultureInfo.InvariantCulture, DateTimeStyles.None, out result))
            {
                return result;
            }

            if (canThrow)
            {
                throw new ArgumentException("Date string is invalid: " + date);
            }

            return default(DateTime);
        }

        public static DateTime GetLdt(DateTime date, TimeZoneInfo timeZone)
        {
            if (timeZone == null)
            {
                return default(DateTime);
            }

            return TimeZoneInfo.ConvertTime(date, timeZone);
        }

        public static uint DaysFromPivotDate(DateTime dateTime)
        {
            if (dateTime <= pivotDate)
            {
                return 0;
            }

            return (uint)(dateTime - pivotDate).TotalDays;
        }

        public static string IsoDateTime(TimeZoneInfo timeZone)
        {
            if (timeZone == null)
            {
                return string.Empty;
            }

            return TimeZoneInfo.ConvertTime(DateTime.UtcNow, timeZone).ToString("yyyy-MM-ddTHH:mm");
        }

        public static ulong SecondsSinceEpoch(string dateTime, TimeZoneInfo timeZone)
        {
            if (string.IsNullOrEmpty(dateTime) || timeZone == null)
            {
                return 0;
            }

            var date = GetFormattedDate(dateTime);
            var utc = TimeZoneInfo.ConvertTimeToUtc(date, timeZone);
            return (ulong)(utc - new DateTime(1970, 1, 1)).TotalSeconds;
        }

        public static int TimezoneDiff(ulong seconds, TimeZoneInfo originTz, TimeZoneInfo destTz)
        {
            if (originTz == null || destTz == null || originTz == destTz)
            {
                return 0;
            }

            var tp = new DateTime(1970, 1, 1).AddSeconds(seconds);
            var origin = TimeZoneInfo.ConvertTime(tp, originTz);
            var dest = TimeZoneInfo.ConvertTime(tp, destTz);

            return (int)(dest - origin).TotalSeconds;
        }

        public static string SecondsToDate(ulong seconds, TimeZoneInfo timeZone, bool tzFormat = false)
        {
            if (seconds == 0 || timeZone == null)
            {
                return string.Empty;
            }

            var tp = new DateTime(1970, 1, 1).AddSeconds(seconds);
            var date = TimeZoneInfo.ConvertTime(tp, timeZone);

            return date.ToString(tzFormat ? "yyyy-MM-ddTHH:mmzzz" : "yyyy-MM-ddTHH:mm");
        }

        public static void SecondsToDate(ulong originSeconds, ulong destSeconds, TimeZoneInfo originTz, TimeZoneInfo destTz, out string isoOrigin, out string isoDest)
        {
            isoOrigin = string.Empty;
            isoDest = string.Empty;

            if (originTz == null || destTz == null)
            {
                return;
            }

            isoOrigin = SecondsToDate(originSeconds, originTz);
            isoDest = SecondsToDate(destSeconds, destTz);
        }

        public static uint DayOfWeekMask(string dateTime)
        {
            var date = GetFormattedDate(dateTime);
            if (date < pivotDate)
            {
                return 0;
            }

            var dayOfWeek = (int)date.DayOfWeek;
            return (uint)(1 << dayOfWeek);
        }

        public static string GetDuration(string dateTime, uint seconds, TimeZoneInfo timeZone)
        {
            var date = GetFormattedDate(dateTime);
            if (date < pivotDate)
            {
                return string.Empty;
            }

            var tp = date.AddSeconds(seconds);
            var origin = TimeZoneInfo.ConvertTime(tp, timeZone);

            return origin.ToString("yyyy-MM-ddTHH:mmzzz");
        }

        public static bool IsConditionalActive(bool type, byte beginHrs, byte beginMins, byte endHrs, byte endMins, byte dow, byte beginWeek, byte beginMonth, byte beginDayDow, byte endWeek, byte endMonth, byte endDayDow, ulong currentTime, TimeZoneInfo timeZone)
        {
            if (timeZone == null)
            {
                return false;
            }

            var tp = new DateTime(1970, 1, 1).AddSeconds(currentTime);
            var inLocalTime = TimeZoneInfo.ConvertTime(tp, timeZone);
            var date = inLocalTime.Date;
            var td = inLocalTime.TimeOfDay;

            var bTd = TimeSpan.FromHours(0);
            var eTd = TimeSpan.FromHours(23).Add(TimeSpan.FromMinutes(59));

            var bMonth = beginMonth;
            var eMonth = endMonth;
            var bDayDow = beginDayDow;
            var eDayDow = endDayDow;
            var bWeek = beginWeek;
            var eWeek = endWeek;

            if (type && beginWeek != 0 && beginDayDow == 0 && beginMonth == 0)
            {
                bMonth = (byte)date.Month;
            }

            if (type && endWeek != 0 && endDayDow == 0 && endMonth == 0)
            {
                eMonth = (byte)date.Month;
            }

            if (type && beginWeek != 0 && beginDayDow == 0 && beginMonth == 0 && endWeek == 0 && endDayDow == 0 && endMonth == 0)
            {
                eMonth = bMonth;
                bDayDow = eDayDow = dow;
                eWeek = bWeek;
            }
            else if (!type && bMonth != 0 && eMonth != 0 && bDayDow == 0 && eDayDow == 0)
            {
                bDayDow = 1;
                eDayDow = (byte)DateTime.DaysInMonth(date.Year, eMonth);
            }

            var edgeCase = false;
            if (!type && bMonth != 0 && eMonth != 0 && bDayDow == 0 && eDayDow == 0 && bWeek == 0 && eWeek == 0 && bMonth == eMonth)
            {
                var dtInRange = bMonth <= date.Month && date.Month <= eMonth;

                if (beginHrs != 0 || beginMins != 0 || endHrs != 0 || endMins != 0)
                {
                    bTd = TimeSpan.FromHours(beginHrs).Add(TimeSpan.FromMinutes(beginMins));
                    eTd = TimeSpan.FromHours(endHrs).Add(TimeSpan.FromMinutes(endMins));
                }

                return dtInRange && bTd <= td && td <= eTd;
            }
            else if (!type && bMonth != 0 && bDayDow != 0)
            {
                var eYear = date.Year;
                var bYear = date.Year;

                if (bMonth == eMonth)
                {
                    if (bDayDow > eDayDow)
                    {
                        edgeCase = true;
                    }
                }
                else if (bMonth > eMonth)
                {
                    if (bMonth > date.Month)
                    {
                        bYear--;
                    }
                    else
                    {
                        eYear++;
                    }
                }

                var beginDate = new DateTime(bYear, bMonth, bDayDow);
                var endDate = new DateTime(eYear, eMonth, eDayDow);

                var bInLocalTime = TimeZoneInfo.ConvertTime(beginDate, timeZone);
                var localDt = TimeZoneInfo.ConvertTime(date, timeZone);
                var eInLocalTime = TimeZoneInfo.ConvertTime(endDate, timeZone);

                if (edgeCase)
                {
                    var newEd = new DateTime(bYear, 12, 31);
                    var newEInLocalTime = TimeZoneInfo.ConvertTime(newEd, timeZone);

                    var newBd = new DateTime(bYear, 1, 1);
                    var newBInLocalTime = TimeZoneInfo.ConvertTime(newBd, timeZone);

                    return (bInLocalTime <= localDt && localDt <= newEInLocalTime) || (newBInLocalTime <= localDt && localDt <= eInLocalTime);
                }
                else
                {
                    return bInLocalTime <= localDt && localDt <= eInLocalTime;
                }
            }
            else
            {
                if (beginHrs != 0 || beginMins != 0 || endHrs != 0 || endMins != 0)
                {
                    bTd = TimeSpan.FromHours(beginHrs).Add(TimeSpan.FromMinutes(beginMins));
                    eTd = TimeSpan.FromHours(endHrs).Add(TimeSpan.FromMinutes(endMins));

                    if (beginHrs > endHrs)
                    {
                        return !(eTd <= td && td <= bTd);
                    }
                    else
                    {
                        return bTd <= td && td <= eTd;
                    }
                }

                return true;
            }
        }

        public static uint SecondOfWeek(uint epochTime, TimeZoneInfo timeZone)
        {
            var tp = new DateTime(1970, 1, 1).AddSeconds(epochTime);
            var localTime = TimeZoneInfo.ConvertTime(tp, timeZone);
            var days = localTime.Date;
            var day = (int)localTime.DayOfWeek;
            var sinceMidnight = (localTime - days).TotalSeconds;

            return (uint)(day * 86400 + sinceMidnight);
        }

        public static (string, string, string) OffsetDate(string inDt, uint inTz, uint outTz, float offset)
        {
            if (string.IsNullOrEmpty(inDt))
            {
                return ("", "", "");
            }

            var iepoch = SecondsSinceEpoch(inDt, TimeZoneInfo.FindSystemTimeZoneById(inTz.ToString()));
            var oepoch = (ulong)(iepoch + offset + 0.5f);
            var tz = TimeZoneInfo.FindSystemTimeZoneById(outTz.ToString());
            var dt = SecondsToDate(oepoch, tz, true);

            if (string.IsNullOrEmpty(dt))
            {
                return ("", "", "");
            }

            return (dt.Substring(0, 16), dt.Substring(16), tz.Id);
        }
    }
}
