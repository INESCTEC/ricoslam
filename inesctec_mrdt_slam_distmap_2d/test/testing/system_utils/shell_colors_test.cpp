#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/system_utils/shell_colors.hpp>

namespace testing
{

TEST(system_utils_shell_colors, ansi_esc_codes)
{
  EXPECT_NO_THROW(std::cout << FG_SC_BLACK("FG_BLACK") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_RED("FG_RED") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_GREEN("FG_GREEN") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_YELLOW("FG_YELLOW") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLUE("FG_BLUE") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BRIGHT_BLACK("FG_BRIGHT_BLACK")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BRIGHT_RED("FG_BRIGHT_RED") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BRIGHT_GREEN("FG_BRIGHT_GREEN")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BRIGHT_YELLOW("FG_BRIGHT_YELLOW")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BRIGHT_BLUE("FG_BRIGHT_BLUE")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BBLACK("FG_BBLACK") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BRED("FG_BRED") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BGREEN("FG_BGREEN") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BYELLOW("FG_BYELLOW") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BBLUE("FG_BBLUE") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BBRIGHT_BLACK("FG_BBRIGHT_BLACK")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BBRIGHT_RED("FG_BBRIGHT_RED")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BBRIGHT_GREEN("FG_BBRIGHT_GREEN")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BBRIGHT_YELLOW("FG_BBRIGHT_YELLOW")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BBRIGHT_BLUE("FG_BBRIGHT_BLUE")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IBLACK("FG_IBLACK") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IRED("FG_IRED") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IGREEN("FG_IGREEN") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IYELLOW("FG_IYELLOW") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IBLUE("FG_IBLUE") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IBRIGHT_BLACK("FG_IBRIGHT_BLACK")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IBRIGHT_RED("FG_IBRIGHT_RED")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IBRIGHT_GREEN("FG_IBRIGHT_GREEN")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IBRIGHT_YELLOW("FG_IBRIGHT_YELLOW")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_IBRIGHT_BLUE("FG_IBRIGHT_BLUE")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UBLACK("FG_UBLACK") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_URED("FG_URED") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UGREEN("FG_UGREEN") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UYELLOW("FG_UYELLOW") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UBLUE("FG_UBLUE") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UBRIGHT_BLACK("FG_UBRIGHT_BLACK")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UBRIGHT_RED("FG_UBRIGHT_RED")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UBRIGHT_GREEN("FG_UBRIGHT_GREEN")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UBRIGHT_YELLOW("FG_UBRIGHT_YELLOW")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_UBRIGHT_BLUE("FG_UBRIGHT_BLUE")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_BLACK("FG_BLINK_BLACK")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_RED("FG_BLINK_RED") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_GREEN("FG_BLINK_GREEN")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_YELLOW("FG_BLINK_YELLOW")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_BLUE("FG_BLINK_BLUE") << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_BRIGHT_BLACK("FG_BLINK_BRIGHT_BLACK")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_BRIGHT_RED("FG_BLINK_BRIGHT_RED")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_BRIGHT_GREEN("FG_BLINK_BRIGHT_GREEN")
                            << std::endl);
  EXPECT_NO_THROW(std::cout
                  << FG_SC_BLINK_BRIGHT_YELLOW("FG_BLINK_BRIGHT_YELLOW")
                  << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_BLINK_BRIGHT_BLUE("FG_BLINK_BRIGHT_BLUE")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_REVERSE_BLACK("FG_REVERSE_BLACK")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_REVERSE_RED("FG_REVERSE_RED")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_REVERSE_GREEN("FG_REVERSE_GREEN")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_REVERSE_YELLOW("FG_REVERSE_YELLOW")
                            << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_REVERSE_BLUE("FG_REVERSE_BLUE")
                            << std::endl);
  EXPECT_NO_THROW(std::cout
                  << FG_SC_REVERSE_BRIGHT_BLACK("FG_REVERSE_BRIGHT_BLACK")
                  << std::endl);
  EXPECT_NO_THROW(std::cout << FG_SC_REVERSE_BRIGHT_RED("FG_REVERSE_BRIGHT_RED")
                            << std::endl);
  EXPECT_NO_THROW(std::cout
                  << FG_SC_REVERSE_BRIGHT_GREEN("FG_REVERSE_BRIGHT_GREEN")
                  << std::endl);
  EXPECT_NO_THROW(std::cout
                  << FG_SC_REVERSE_BRIGHT_YELLOW("FG_REVERSE_BRIGHT_YELLOW")
                  << std::endl);
  EXPECT_NO_THROW(std::cout
                  << FG_SC_REVERSE_BRIGHT_BLUE("FG_REVERSE_BRIGHT_BLUE")
                  << std::endl);
}

}  // namespace testing
