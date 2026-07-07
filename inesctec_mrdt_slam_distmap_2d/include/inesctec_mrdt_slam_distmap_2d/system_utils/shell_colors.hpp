#pragma once

namespace inesctec_mrdt_slam_distmap_2d
{

namespace system_utils
{

/**
 * @brief Sources ANSI Escape codes
 *
 * https://en.wikipedia.org/wiki/ANSI_escape_code
 * https://gist.github.com/Kielx/2917687bc30f567d45e15a4577772b02
 * https://gitlab.com/srrg-software/srrg2_core/-/blob/main/srrg2_core/src/srrg_system_utils/shell_colors.h
 *
 * Note: these constants definitions could be further automatized and
 * summarized....
 * also, you can even make more combinations with these...
 */

#define SC_RESET "\033[0m"

//! standard colors

#define SC_BLACK "\033[30m"
#define SC_RED "\033[31m"
#define SC_GREEN "\033[32m"
#define SC_YELLOW "\033[33m"
#define SC_BLUE "\033[34m"
#define SC_MAGENTA "\033[35m"
#define SC_CYAN "\033[36m"
#define SC_WHITE "\033[37m"

#define SC_BRIGHT_BLACK "\033[90m"
#define SC_BRIGHT_RED "\033[91m"
#define SC_BRIGHT_GREEN "\033[92m"
#define SC_BRIGHT_YELLOW "\033[93m"
#define SC_BRIGHT_BLUE "\033[94m"
#define SC_BRIGHT_MAGENTA "\033[95m"
#define SC_BRIGHT_CYAN "\033[96m"
#define SC_BRIGHT_WHITE "\033[97m"

//! bold

#define SC_BBLACK "\033[1;30m"    // or "\033[1m\033[30m"
#define SC_BRED "\033[1;31m"      // or "\033[1m\033[31m"
#define SC_BGREEN "\033[1;32m"    // or "\033[1m\033[32m"
#define SC_BYELLOW "\033[1;33m"   // or "\033[1m\033[33m"
#define SC_BBLUE "\033[1;34m"     // or "\033[1m\033[34m"
#define SC_BMAGENTA "\033[1;35m"  // or "\033[1m\033[35m"
#define SC_BCYAN "\033[1;36m"     // or "\033[1m\033[36m"
#define SC_BWHITE "\033[1;37m"    // or "\033[1m\033[37m"

#define SC_BBRIGHT_BLACK "\033[1;90m"
#define SC_BBRIGHT_RED "\033[1;91m"
#define SC_BBRIGHT_GREEN "\033[1;92m"
#define SC_BBRIGHT_YELLOW "\033[1;93m"
#define SC_BBRIGHT_BLUE "\033[1;94m"
#define SC_BBRIGHT_MAGENTA "\033[1;95m"
#define SC_BBRIGHT_CYAN "\033[1;96m"
#define SC_BBRIGHT_WHITE "\033[1;97m"

//! italic

#define SC_IBLACK "\033[3;30m"
#define SC_IRED "\033[3;31m"
#define SC_IGREEN "\033[3;32m"
#define SC_IYELLOW "\033[3;33m"
#define SC_IBLUE "\033[3;34m"
#define SC_IMAGENTA "\033[3;35m"
#define SC_ICYAN "\033[3;36m"
#define SC_IWHITE "\033[3;37m"

#define SC_IBRIGHT_BLACK "\033[3;90m"
#define SC_IBRIGHT_RED "\033[3;91m"
#define SC_IBRIGHT_GREEN "\033[3;92m"
#define SC_IBRIGHT_YELLOW "\033[3;93m"
#define SC_IBRIGHT_BLUE "\033[3;94m"
#define SC_IBRIGHT_MAGENTA "\033[3;95m"
#define SC_IBRIGHT_CYAN "\033[3;96m"
#define SC_IBRIGHT_WHITE "\033[3;97m"

//! underline

#define SC_UBLACK "\033[4;30m"
#define SC_URED "\033[4;31m"
#define SC_UGREEN "\033[4;32m"
#define SC_UYELLOW "\033[4;33m"
#define SC_UBLUE "\033[4;34m"
#define SC_UMAGENTA "\033[4;35m"
#define SC_UCYAN "\033[4;36m"
#define SC_UWHITE "\033[4;37m"

#define SC_UBRIGHT_BLACK "\033[4;90m"
#define SC_UBRIGHT_RED "\033[4;91m"
#define SC_UBRIGHT_GREEN "\033[4;92m"
#define SC_UBRIGHT_YELLOW "\033[4;93m"
#define SC_UBRIGHT_BLUE "\033[4;94m"
#define SC_UBRIGHT_MAGENTA "\033[4;95m"
#define SC_UBRIGHT_CYAN "\033[4;96m"
#define SC_UBRIGHT_WHITE "\033[4;97m"

//! blink

#define SC_BLINK_BLACK "\033[5;30m"
#define SC_BLINK_RED "\033[5;31m"
#define SC_BLINK_GREEN "\033[5;32m"
#define SC_BLINK_YELLOW "\033[5;33m"
#define SC_BLINK_BLUE "\033[5;34m"
#define SC_BLINK_MAGENTA "\033[5;35m"
#define SC_BLINK_CYAN "\033[5;36m"
#define SC_BLINK_WHITE "\033[5;37m"

#define SC_BLINK_BRIGHT_BLACK "\033[5;90m"
#define SC_BLINK_BRIGHT_RED "\033[5;91m"
#define SC_BLINK_BRIGHT_GREEN "\033[5;92m"
#define SC_BLINK_BRIGHT_YELLOW "\033[5;93m"
#define SC_BLINK_BRIGHT_BLUE "\033[5;94m"
#define SC_BLINK_BRIGHT_MAGENTA "\033[5;95m"
#define SC_BLINK_BRIGHT_CYAN "\033[5;96m"
#define SC_BLINK_BRIGHT_WHITE "\033[5;97m"

//! reverse colors (background <> foreground)

#define SC_REVERSE_BLACK "\033[7;30m"
#define SC_REVERSE_RED "\033[7;31m"
#define SC_REVERSE_GREEN "\033[7;32m"
#define SC_REVERSE_YELLOW "\033[7;33m"
#define SC_REVERSE_BLUE "\033[7;34m"
#define SC_REVERSE_MAGENTA "\033[7;35m"
#define SC_REVERSE_CYAN "\033[7;36m"
#define SC_REVERSE_WHITE "\033[7;37m"

#define SC_REVERSE_BRIGHT_BLACK "\033[7;90m"
#define SC_REVERSE_BRIGHT_RED "\033[7;91m"
#define SC_REVERSE_BRIGHT_GREEN "\033[7;92m"
#define SC_REVERSE_BRIGHT_YELLOW "\033[7;93m"
#define SC_REVERSE_BRIGHT_BLUE "\033[7;94m"
#define SC_REVERSE_BRIGHT_MAGENTA "\033[7;95m"
#define SC_REVERSE_BRIGHT_CYAN "\033[7;96m"
#define SC_REVERSE_BRIGHT_WHITE "\033[7;97m"

//! std::cout << FG_<style>(string) << std::endl;

#define FG_SC_BLACK(str) SC_BLACK << str << SC_RESET
#define FG_SC_RED(str) SC_RED << str << SC_RESET
#define FG_SC_GREEN(str) SC_GREEN << str << SC_RESET
#define FG_SC_YELLOW(str) SC_YELLOW << str << SC_RESET
#define FG_SC_BLUE(str) SC_BLUE << str << SC_RESET
#define FG_SC_MAGENTA(str) SC_MAGENTA << str << SC_RESET
#define FG_SC_CYAN(str) SC_CYAN << str << SC_RESET
#define FG_SC_WHITE(str) SC_WHITE << str << SC_RESET

#define FG_SC_BRIGHT_BLACK(str) SC_BRIGHT_BLACK << str << SC_RESET
#define FG_SC_BRIGHT_RED(str) SC_BRIGHT_RED << str << SC_RESET
#define FG_SC_BRIGHT_GREEN(str) SC_BRIGHT_GREEN << str << SC_RESET
#define FG_SC_BRIGHT_YELLOW(str) SC_BRIGHT_YELLOW << str << SC_RESET
#define FG_SC_BRIGHT_BLUE(str) SC_BRIGHT_BLUE << str << SC_RESET
#define FG_SC_BRIGHT_MAGENTA(str) SC_BRIGHT_MAGENTA << str << SC_RESET
#define FG_SC_BRIGHT_CYAN(str) SC_BRIGHT_CYAN << str << SC_RESET
#define FG_SC_BRIGHT_WHITE(str) SC_BRIGHT_WHITE << str << SC_RESET

#define FG_SC_BBLACK(str) SC_BBLACK << str << SC_RESET
#define FG_SC_BRED(str) SC_BRED << str << SC_RESET
#define FG_SC_BGREEN(str) SC_BGREEN << str << SC_RESET
#define FG_SC_BYELLOW(str) SC_BYELLOW << str << SC_RESET
#define FG_SC_BBLUE(str) SC_BBLUE << str << SC_RESET
#define FG_SC_BMAGENTA(str) SC_BMAGENTA << str << SC_RESET
#define FG_SC_BCYAN(str) SC_BCYAN << str << SC_RESET
#define FG_SC_BWHITE(str) SC_BWHITE << str << SC_RESET

#define FG_SC_BBRIGHT_BLACK(str) SC_BBRIGHT_BLACK << str << SC_RESET
#define FG_SC_BBRIGHT_RED(str) SC_BBRIGHT_RED << str << SC_RESET
#define FG_SC_BBRIGHT_GREEN(str) SC_BBRIGHT_GREEN << str << SC_RESET
#define FG_SC_BBRIGHT_YELLOW(str) SC_BBRIGHT_YELLOW << str << SC_RESET
#define FG_SC_BBRIGHT_BLUE(str) SC_BBRIGHT_BLUE << str << SC_RESET
#define FG_SC_BBRIGHT_MAGENTA(str) SC_BBRIGHT_MAGENTA << str << SC_RESET
#define FG_SC_BBRIGHT_CYAN(str) SC_BBRIGHT_CYAN << str << SC_RESET
#define FG_SC_BBRIGHT_WHITE(str) SC_BBRIGHT_WHITE << str << SC_RESET

#define FG_SC_IBLACK(str) SC_IBLACK << str << SC_RESET
#define FG_SC_IRED(str) SC_IRED << str << SC_RESET
#define FG_SC_IGREEN(str) SC_IGREEN << str << SC_RESET
#define FG_SC_IYELLOW(str) SC_IYELLOW << str << SC_RESET
#define FG_SC_IBLUE(str) SC_IBLUE << str << SC_RESET
#define FG_SC_IMAGENTA(str) SC_IMAGENTA << str << SC_RESET
#define FG_SC_ICYAN(str) SC_ICYAN << str << SC_RESET
#define FG_SC_IWHITE(str) SC_IWHITE << str << SC_RESET

#define FG_SC_IBRIGHT_BLACK(str) SC_IBRIGHT_BLACK << str << SC_RESET
#define FG_SC_IBRIGHT_RED(str) SC_IBRIGHT_RED << str << SC_RESET
#define FG_SC_IBRIGHT_GREEN(str) SC_IBRIGHT_GREEN << str << SC_RESET
#define FG_SC_IBRIGHT_YELLOW(str) SC_IBRIGHT_YELLOW << str << SC_RESET
#define FG_SC_IBRIGHT_BLUE(str) SC_IBRIGHT_BLUE << str << SC_RESET
#define FG_SC_IBRIGHT_MAGENTA(str) SC_IBRIGHT_MAGENTA << str << SC_RESET
#define FG_SC_IBRIGHT_CYAN(str) SC_IBRIGHT_CYAN << str << SC_RESET
#define FG_SC_IBRIGHT_WHITE(str) SC_IBRIGHT_WHITE << str << SC_RESET

#define FG_SC_UBLACK(str) SC_UBLACK << str << SC_RESET
#define FG_SC_URED(str) SC_URED << str << SC_RESET
#define FG_SC_UGREEN(str) SC_UGREEN << str << SC_RESET
#define FG_SC_UYELLOW(str) SC_UYELLOW << str << SC_RESET
#define FG_SC_UBLUE(str) SC_UBLUE << str << SC_RESET
#define FG_SC_UMAGENTA(str) SC_UMAGENTA << str << SC_RESET
#define FG_SC_UCYAN(str) SC_UCYAN << str << SC_RESET
#define FG_SC_UWHITE(str) SC_UWHITE << str << SC_RESET

#define FG_SC_UBRIGHT_BLACK(str) SC_UBRIGHT_BLACK << str << SC_RESET
#define FG_SC_UBRIGHT_RED(str) SC_UBRIGHT_RED << str << SC_RESET
#define FG_SC_UBRIGHT_GREEN(str) SC_UBRIGHT_GREEN << str << SC_RESET
#define FG_SC_UBRIGHT_YELLOW(str) SC_UBRIGHT_YELLOW << str << SC_RESET
#define FG_SC_UBRIGHT_BLUE(str) SC_UBRIGHT_BLUE << str << SC_RESET
#define FG_SC_UBRIGHT_MAGENTA(str) SC_UBRIGHT_MAGENTA << str << SC_RESET
#define FG_SC_UBRIGHT_CYAN(str) SC_UBRIGHT_CYAN << str << SC_RESET
#define FG_SC_UBRIGHT_WHITE(str) SC_UBRIGHT_WHITE << str << SC_RESET

#define FG_SC_BLINK_BLACK(str) SC_BLINK_BLACK << str << SC_RESET
#define FG_SC_BLINK_RED(str) SC_BLINK_RED << str << SC_RESET
#define FG_SC_BLINK_GREEN(str) SC_BLINK_GREEN << str << SC_RESET
#define FG_SC_BLINK_YELLOW(str) SC_BLINK_YELLOW << str << SC_RESET
#define FG_SC_BLINK_BLUE(str) SC_BLINK_BLUE << str << SC_RESET
#define FG_SC_BLINK_MAGENTA(str) SC_BLINK_MAGENTA << str << SC_RESET
#define FG_SC_BLINK_CYAN(str) SC_BLINK_CYAN << str << SC_RESET
#define FG_SC_BLINK_WHITE(str) SC_BLINK_WHITE << str << SC_RESET

#define FG_SC_BLINK_BRIGHT_BLACK(str) SC_BLINK_BRIGHT_BLACK << str << SC_RESET
#define FG_SC_BLINK_BRIGHT_RED(str) SC_BLINK_BRIGHT_RED << str << SC_RESET
#define FG_SC_BLINK_BRIGHT_GREEN(str) SC_BLINK_BRIGHT_GREEN << str << SC_RESET
#define FG_SC_BLINK_BRIGHT_YELLOW(str) SC_BLINK_BRIGHT_YELLOW << str << SC_RESET
#define FG_SC_BLINK_BRIGHT_BLUE(str) SC_BLINK_BRIGHT_BLUE << str << SC_RESET
#define FG_SC_BLINK_BRIGHT_MAGENTA(str) \
  SC_BLINK_BRIGHT_MAGENTA << str << SC_RESET
#define FG_SC_BLINK_BRIGHT_CYAN(str) SC_BLINK_BRIGHT_CYAN << str << SC_RESET
#define FG_SC_BLINK_BRIGHT_WHITE(str) SC_BLINK_BRIGHT_WHITE << str << SC_RESET

#define FG_SC_REVERSE_BLACK(str) SC_REVERSE_BLACK << str << SC_RESET
#define FG_SC_REVERSE_RED(str) SC_REVERSE_RED << str << SC_RESET
#define FG_SC_REVERSE_GREEN(str) SC_REVERSE_GREEN << str << SC_RESET
#define FG_SC_REVERSE_YELLOW(str) SC_REVERSE_YELLOW << str << SC_RESET
#define FG_SC_REVERSE_BLUE(str) SC_REVERSE_BLUE << str << SC_RESET
#define FG_SC_REVERSE_MAGENTA(str) SC_REVERSE_MAGENTA << str << SC_RESET
#define FG_SC_REVERSE_CYAN(str) SC_REVERSE_CYAN << str << SC_RESET
#define FG_SC_REVERSE_WHITE(str) SC_REVERSE_WHITE << str << SC_RESET

#define FG_SC_REVERSE_BRIGHT_BLACK(str) \
  SC_REVERSE_BRIGHT_BLACK << str << SC_RESET
#define FG_SC_REVERSE_BRIGHT_RED(str) SC_REVERSE_BRIGHT_RED << str << SC_RESET
#define FG_SC_REVERSE_BRIGHT_GREEN(str) \
  SC_REVERSE_BRIGHT_GREEN << str << SC_RESET
#define FG_SC_REVERSE_BRIGHT_YELLOW(str) \
  SC_REVERSE_BRIGHT_YELLOW << str << SC_RESET
#define FG_SC_REVERSE_BRIGHT_BLUE(str) SC_REVERSE_BRIGHT_BLUE << str << SC_RESET
#define FG_SC_REVERSE_BRIGHT_MAGENTA(str) \
  SC_REVERSE_BRIGHT_MAGENTA << str << SC_RESET
#define FG_SC_REVERSE_BRIGHT_CYAN(str) SC_REVERSE_BRIGHT_CYAN << str << SC_RESET
#define FG_SC_REVERSE_BRIGHT_WHITE(str) \
  SC_REVERSE_BRIGHT_WHITE << str << SC_RESET

}  // namespace system_utils

}  // namespace inesctec_mrdt_slam_distmap_2d
