#include <gtest/gtest.h>

#include <string>

TEST(PreloaderTests, ThreadTester)
{

    const auto expected = 1;
    const auto actual = 1;
    ASSERT_EQ(expected, actual);
    std::cout << "ThreadTester done" << std::endl;
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}